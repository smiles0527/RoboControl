using System.Collections.Concurrent;
using MathNet.Numerics.LinearAlgebra;
using SysMath = System.Math;

namespace ControlWorkbench.Drone.Enterprise;

/// <summary>
/// Enterprise Event Bus for Drone Operations.
/// Provides pub/sub messaging, event sourcing, and integration with enterprise systems.
/// 
/// Features:
/// - High-throughput event streaming
/// - Event sourcing with replay capability
/// - Dead letter queue for failed processing
/// - Integration with enterprise message brokers
/// - Event filtering and routing
/// </summary>
public class EnterpriseEventBus : IDisposable
{
    private readonly EventBusConfig _config;
    private readonly ConcurrentDictionary<string, List<IEventHandler>> _handlers = new();
    private readonly ConcurrentDictionary<string, EventStream> _streams = new();
    private readonly ConcurrentQueue<EventEnvelope> _pendingEvents = new();
    private readonly ConcurrentQueue<DeadLetterEvent> _deadLetterQueue = new();
    private readonly EventStore _eventStore;
    
    private CancellationTokenSource? _cts;
    private Task? _processingTask;
    
    public event Action<EventEnvelope>? EventPublished;
    public event Action<DeadLetterEvent>? DeadLetterQueued;
    
    public long TotalEventsProcessed { get; private set; }
    public int PendingEventCount => _pendingEvents.Count;
    public int DeadLetterCount => _deadLetterQueue.Count;
    
    public EnterpriseEventBus(EventBusConfig? config = null)
    {
        _config = config ?? new EventBusConfig();
        _eventStore = new EventStore(_config);
    }
    
    /// <summary>
    /// Start the event bus processing loop.
    /// </summary>
    public void Start()
    {
        _cts = new CancellationTokenSource();
        _processingTask = Task.Run(() => ProcessEventsLoop(_cts.Token), _cts.Token);
    }
    
    /// <summary>
    /// Stop the event bus.
    /// </summary>
    public async Task StopAsync()
    {
        _cts?.Cancel();
        if (_processingTask != null)
        {
            try { await _processingTask; } catch { }
        }
    }
    
    /// <summary>
    /// Subscribe to events of a specific type.
    /// </summary>
    public IDisposable Subscribe<TEvent>(Action<TEvent> handler) where TEvent : class
    {
        var typeName = typeof(TEvent).Name;
        var wrapper = new TypedEventHandler<TEvent>(handler);
        
        if (!_handlers.ContainsKey(typeName))
        {
            _handlers[typeName] = new List<IEventHandler>();
        }
        
        _handlers[typeName].Add(wrapper);
        
        return new Subscription(() =>
        {
            if (_handlers.TryGetValue(typeName, out var list))
            {
                list.Remove(wrapper);
            }
        });
    }
    
    /// <summary>
    /// Subscribe to a specific event stream.
    /// </summary>
    public IDisposable SubscribeToStream(string streamName, Action<EventEnvelope> handler)
    {
        if (!_streams.ContainsKey(streamName))
        {
            _streams[streamName] = new EventStream(streamName);
        }
        
        _streams[streamName].Handlers.Add(handler);
        
        return new Subscription(() =>
        {
            if (_streams.TryGetValue(streamName, out var stream))
            {
                stream.Handlers.Remove(handler);
            }
        });
    }
    
    /// <summary>
    /// Publish an event.
    /// </summary>
    public void Publish<TEvent>(TEvent evt, PublishOptions? options = null) where TEvent : class
    {
        var envelope = new EventEnvelope
        {
            Id = Guid.NewGuid().ToString(),
            Timestamp = DateTime.UtcNow,
            EventType = typeof(TEvent).Name,
            Payload = evt,
            StreamName = options?.StreamName,
            CorrelationId = options?.CorrelationId ?? Guid.NewGuid().ToString(),
            CausationId = options?.CausationId,
            Metadata = options?.Metadata ?? new Dictionary<string, string>()
        };
        
        _pendingEvents.Enqueue(envelope);
        
        // Store in event store if configured
        if (_config.EnableEventSourcing)
        {
            _eventStore.Append(envelope);
        }
        
        EventPublished?.Invoke(envelope);
    }
    
    /// <summary>
    /// Publish to a specific stream.
    /// </summary>
    public void PublishToStream(string streamName, object evt, string? correlationId = null)
    {
        Publish(evt, new PublishOptions
        {
            StreamName = streamName,
            CorrelationId = correlationId
        });
    }
    
    /// <summary>
    /// Replay events from a stream.
    /// </summary>
    public async Task ReplayStreamAsync(
        string streamName,
        DateTime? fromTimestamp = null,
        Action<EventEnvelope>? handler = null,
        CancellationToken ct = default)
    {
        var events = await _eventStore.GetStreamEventsAsync(streamName, fromTimestamp, ct);
        
        foreach (var evt in events)
        {
            if (handler != null)
            {
                handler(evt);
            }
            else
            {
                await DispatchEventAsync(evt);
            }
        }
    }
    
    /// <summary>
    /// Get aggregate state from event stream.
    /// </summary>
    public async Task<TAggregate> RehydrateAggregateAsync<TAggregate>(
        string aggregateId,
        Func<TAggregate> factory,
        Action<TAggregate, EventEnvelope> apply,
        CancellationToken ct = default) where TAggregate : class
    {
        var aggregate = factory();
        var events = await _eventStore.GetAggregateEventsAsync(aggregateId, ct);
        
        foreach (var evt in events)
        {
            apply(aggregate, evt);
        }
        
        return aggregate;
    }
    
    /// <summary>
    /// Process dead letter queue.
    /// </summary>
    public async Task ProcessDeadLettersAsync(
        Func<DeadLetterEvent, Task<bool>> handler,
        CancellationToken ct = default)
    {
        var batch = new List<DeadLetterEvent>();
        
        while (_deadLetterQueue.TryDequeue(out var evt) && batch.Count < 100)
        {
            batch.Add(evt);
        }
        
        foreach (var evt in batch)
        {
            try
            {
                var processed = await handler(evt);
                if (!processed)
                {
                    // Re-queue if not processed
                    _deadLetterQueue.Enqueue(evt);
                }
            }
            catch
            {
                // Back to dead letter
                evt.RetryCount++;
                _deadLetterQueue.Enqueue(evt);
            }
        }
    }
    
    private async Task ProcessEventsLoop(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                while (_pendingEvents.TryDequeue(out var envelope))
                {
                    await DispatchEventAsync(envelope);
                    TotalEventsProcessed++;
                }
                
                await Task.Delay(1, ct);
            }
            catch (OperationCanceledException) { break; }
        }
    }
    
    private async Task DispatchEventAsync(EventEnvelope envelope)
    {
        var dispatched = false;
        
        // Dispatch to typed handlers
        if (_handlers.TryGetValue(envelope.EventType, out var handlers))
        {
            foreach (var handler in handlers.ToList())
            {
                try
                {
                    await handler.HandleAsync(envelope.Payload);
                    dispatched = true;
                }
                catch (Exception ex)
                {
                    HandleDispatchError(envelope, ex);
                }
            }
        }
        
        // Dispatch to stream handlers
        if (!string.IsNullOrEmpty(envelope.StreamName) &&
            _streams.TryGetValue(envelope.StreamName, out var stream))
        {
            foreach (var handler in stream.Handlers.ToList())
            {
                try
                {
                    handler(envelope);
                    dispatched = true;
                }
                catch (Exception ex)
                {
                    HandleDispatchError(envelope, ex);
                }
            }
        }
        
        // If not dispatched, send to dead letter if configured
        if (!dispatched && _config.SendUnhandledToDeadLetter)
        {
            _deadLetterQueue.Enqueue(new DeadLetterEvent
            {
                OriginalEvent = envelope,
                Reason = "No handlers registered",
                Timestamp = DateTime.UtcNow
            });
        }
    }
    
    private void HandleDispatchError(EventEnvelope envelope, Exception ex)
    {
        var deadLetter = new DeadLetterEvent
        {
            OriginalEvent = envelope,
            Reason = ex.Message,
            Exception = ex.ToString(),
            Timestamp = DateTime.UtcNow
        };
        
        _deadLetterQueue.Enqueue(deadLetter);
        DeadLetterQueued?.Invoke(deadLetter);
    }
    
    public void Dispose()
    {
        _cts?.Cancel();
        _eventStore.Dispose();
    }
}

/// <summary>
/// Durable event store for event sourcing.
/// </summary>
public class EventStore : IDisposable
{
    private readonly EventBusConfig _config;
    private readonly ConcurrentDictionary<string, List<EventEnvelope>> _streamEvents = new();
    private readonly ConcurrentDictionary<string, List<EventEnvelope>> _aggregateEvents = new();
    private readonly List<EventEnvelope> _allEvents = new();
    private readonly object _lock = new();
    private StreamWriter? _journalWriter;
    
    public EventStore(EventBusConfig config)
    {
        _config = config;
        
        if (!string.IsNullOrEmpty(config.JournalPath))
        {
            Directory.CreateDirectory(Path.GetDirectoryName(config.JournalPath)!);
            _journalWriter = new StreamWriter(config.JournalPath, append: true);
        }
    }
    
    public void Append(EventEnvelope envelope)
    {
        lock (_lock)
        {
            _allEvents.Add(envelope);
            
            if (!string.IsNullOrEmpty(envelope.StreamName))
            {
                if (!_streamEvents.ContainsKey(envelope.StreamName))
                    _streamEvents[envelope.StreamName] = new List<EventEnvelope>();
                _streamEvents[envelope.StreamName].Add(envelope);
            }
            
            if (envelope.Metadata.TryGetValue("AggregateId", out var aggregateId))
            {
                if (!_aggregateEvents.ContainsKey(aggregateId))
                    _aggregateEvents[aggregateId] = new List<EventEnvelope>();
                _aggregateEvents[aggregateId].Add(envelope);
            }
            
            // Write to journal
            _journalWriter?.WriteLine(System.Text.Json.JsonSerializer.Serialize(envelope));
            _journalWriter?.Flush();
        }
    }
    
    public Task<IReadOnlyList<EventEnvelope>> GetStreamEventsAsync(
        string streamName,
        DateTime? fromTimestamp = null,
        CancellationToken ct = default)
    {
        lock (_lock)
        {
            if (!_streamEvents.TryGetValue(streamName, out var events))
                return Task.FromResult<IReadOnlyList<EventEnvelope>>(new List<EventEnvelope>());
            
            var filtered = fromTimestamp.HasValue
                ? events.Where(e => e.Timestamp >= fromTimestamp.Value).ToList()
                : events.ToList();
            
            return Task.FromResult<IReadOnlyList<EventEnvelope>>(filtered);
        }
    }
    
    public Task<IReadOnlyList<EventEnvelope>> GetAggregateEventsAsync(
        string aggregateId,
        CancellationToken ct = default)
    {
        lock (_lock)
        {
            if (!_aggregateEvents.TryGetValue(aggregateId, out var events))
                return Task.FromResult<IReadOnlyList<EventEnvelope>>(new List<EventEnvelope>());
            
            return Task.FromResult<IReadOnlyList<EventEnvelope>>(events.ToList());
        }
    }
    
    public void Dispose()
    {
        _journalWriter?.Dispose();
    }
}

/// <summary>
/// Multi-tenant fleet operations manager.
/// </summary>
public class MultiTenantFleetManager
{
    private readonly ConcurrentDictionary<string, TenantContext> _tenants = new();
    private readonly EnterpriseEventBus _eventBus;
    private readonly TenantIsolationPolicy _isolationPolicy;
    
    public MultiTenantFleetManager(EnterpriseEventBus eventBus, TenantIsolationPolicy? policy = null)
    {
        _eventBus = eventBus;
        _isolationPolicy = policy ?? new TenantIsolationPolicy();
    }
    
    /// <summary>
    /// Register a new tenant.
    /// </summary>
    public async Task<TenantContext> RegisterTenantAsync(TenantRegistration registration)
    {
        var tenant = new TenantContext
        {
            TenantId = Guid.NewGuid().ToString(),
            Name = registration.Name,
            Plan = registration.Plan,
            CreatedAt = DateTime.UtcNow,
            Limits = GetLimitsForPlan(registration.Plan),
            ApiKey = GenerateApiKey(),
            SecretKey = GenerateSecretKey()
        };
        
        _tenants[tenant.TenantId] = tenant;
        
        _eventBus.Publish(new TenantCreatedEvent
        {
            TenantId = tenant.TenantId,
            Name = tenant.Name,
            Plan = tenant.Plan
        }, new PublishOptions { StreamName = "tenant-events" });
        
        return tenant;
    }
    
    /// <summary>
    /// Get tenant context by API key.
    /// </summary>
    public TenantContext? GetTenantByApiKey(string apiKey)
    {
        return _tenants.Values.FirstOrDefault(t => t.ApiKey == apiKey);
    }
    
    /// <summary>
    /// Validate tenant access to a vehicle.
    /// </summary>
    public bool ValidateVehicleAccess(string tenantId, string vehicleId)
    {
        if (!_tenants.TryGetValue(tenantId, out var tenant))
            return false;
        
        return tenant.Vehicles.Contains(vehicleId);
    }
    
    /// <summary>
    /// Add vehicle to tenant.
    /// </summary>
    public async Task<bool> AddVehicleToTenantAsync(string tenantId, string vehicleId)
    {
        if (!_tenants.TryGetValue(tenantId, out var tenant))
            return false;
        
        if (tenant.Vehicles.Count >= tenant.Limits.MaxVehicles)
            return false;
        
        tenant.Vehicles.Add(vehicleId);
        
        _eventBus.Publish(new VehicleAddedToTenantEvent
        {
            TenantId = tenantId,
            VehicleId = vehicleId
        }, new PublishOptions { StreamName = "tenant-events" });
        
        return true;
    }
    
    /// <summary>
    /// Get tenant usage metrics.
    /// </summary>
    public TenantUsageMetrics GetUsageMetrics(string tenantId)
    {
        if (!_tenants.TryGetValue(tenantId, out var tenant))
            return new TenantUsageMetrics();
        
        return new TenantUsageMetrics
        {
            TenantId = tenantId,
            VehicleCount = tenant.Vehicles.Count,
            VehicleLimit = tenant.Limits.MaxVehicles,
            PilotCount = tenant.Pilots.Count,
            PilotLimit = tenant.Limits.MaxPilots,
            MonthlyFlightHours = tenant.MonthlyFlightHours,
            FlightHoursLimit = tenant.Limits.MaxMonthlyFlightHours,
            StorageUsedGb = tenant.StorageUsedGb,
            StorageLimitGb = tenant.Limits.MaxStorageGb,
            ApiCallsThisMonth = tenant.ApiCallsThisMonth,
            ApiCallLimit = tenant.Limits.MaxApiCallsPerMonth
        };
    }
    
    /// <summary>
    /// Check if operation is allowed by tenant limits.
    /// </summary>
    public TenantOperationResult CheckOperationAllowed(
        string tenantId,
        TenantOperation operation)
    {
        if (!_tenants.TryGetValue(tenantId, out var tenant))
            return new TenantOperationResult { Allowed = false, Reason = "Tenant not found" };
        
        switch (operation)
        {
            case TenantOperation.StartFlight:
                if (tenant.ActiveFlights >= tenant.Limits.MaxConcurrentFlights)
                    return new TenantOperationResult { Allowed = false, Reason = "Concurrent flight limit reached" };
                break;
            
            case TenantOperation.AddVehicle:
                if (tenant.Vehicles.Count >= tenant.Limits.MaxVehicles)
                    return new TenantOperationResult { Allowed = false, Reason = "Vehicle limit reached" };
                break;
            
            case TenantOperation.AddPilot:
                if (tenant.Pilots.Count >= tenant.Limits.MaxPilots)
                    return new TenantOperationResult { Allowed = false, Reason = "Pilot limit reached" };
                break;
            
            case TenantOperation.ApiCall:
                if (tenant.ApiCallsThisMonth >= tenant.Limits.MaxApiCallsPerMonth)
                    return new TenantOperationResult { Allowed = false, Reason = "API call limit reached" };
                break;
        }
        
        return new TenantOperationResult { Allowed = true };
    }
    
    private TenantLimits GetLimitsForPlan(TenantPlan plan) => plan switch
    {
        TenantPlan.Starter => new TenantLimits
        {
            MaxVehicles = 5,
            MaxPilots = 10,
            MaxConcurrentFlights = 1,
            MaxMonthlyFlightHours = 20,
            MaxStorageGb = 10,
            MaxApiCallsPerMonth = 10000
        },
        TenantPlan.Professional => new TenantLimits
        {
            MaxVehicles = 25,
            MaxPilots = 50,
            MaxConcurrentFlights = 5,
            MaxMonthlyFlightHours = 100,
            MaxStorageGb = 100,
            MaxApiCallsPerMonth = 100000
        },
        TenantPlan.Enterprise => new TenantLimits
        {
            MaxVehicles = int.MaxValue,
            MaxPilots = int.MaxValue,
            MaxConcurrentFlights = 100,
            MaxMonthlyFlightHours = double.MaxValue,
            MaxStorageGb = 10000,
            MaxApiCallsPerMonth = int.MaxValue
        },
        _ => new TenantLimits()
    };
    
    private string GenerateApiKey() => $"cwb_{Guid.NewGuid():N}";
    private string GenerateSecretKey() => Convert.ToBase64String(
        System.Security.Cryptography.RandomNumberGenerator.GetBytes(32));
}

/// <summary>
/// Real-time video and data streaming service.
/// </summary>
public class VideoStreamingService : IDisposable
{
    private readonly StreamingConfig _config;
    private readonly ConcurrentDictionary<string, VideoStream> _activeStreams = new();
    private readonly ConcurrentDictionary<string, List<IStreamConsumer>> _consumers = new();
    
    public event Action<VideoStreamStartedEvent>? StreamStarted;
    public event Action<VideoStreamEndedEvent>? StreamEnded;
    
    public VideoStreamingService(StreamingConfig? config = null)
    {
        _config = config ?? new StreamingConfig();
    }
    
    /// <summary>
    /// Start a video stream from a vehicle.
    /// </summary>
    public async Task<VideoStream> StartStreamAsync(
        StartStreamRequest request,
        CancellationToken ct = default)
    {
        var stream = new VideoStream
        {
            StreamId = Guid.NewGuid().ToString(),
            VehicleId = request.VehicleId,
            Quality = request.Quality,
            Protocol = request.Protocol,
            StartedAt = DateTime.UtcNow,
            Status = StreamStatus.Starting
        };
        
        // Configure encoding based on quality
        stream.Encoder = ConfigureEncoder(request.Quality);
        
        // Generate stream URLs
        stream.RtmpUrl = $"{_config.RtmpEndpoint}/{stream.StreamId}";
        stream.HlsUrl = $"{_config.HlsEndpoint}/{stream.StreamId}/playlist.m3u8";
        stream.WebRtcUrl = $"{_config.WebRtcEndpoint}/{stream.StreamId}";
        
        _activeStreams[stream.StreamId] = stream;
        stream.Status = StreamStatus.Active;
        
        StreamStarted?.Invoke(new VideoStreamStartedEvent
        {
            StreamId = stream.StreamId,
            VehicleId = request.VehicleId
        });
        
        return stream;
    }
    
    /// <summary>
    /// Stop a video stream.
    /// </summary>
    public async Task StopStreamAsync(string streamId)
    {
        if (_activeStreams.TryRemove(streamId, out var stream))
        {
            stream.Status = StreamStatus.Stopped;
            stream.EndedAt = DateTime.UtcNow;
            
            // Notify consumers
            if (_consumers.TryRemove(streamId, out var consumers))
            {
                foreach (var consumer in consumers)
                {
                    consumer.OnStreamEnded();
                }
            }
            
            StreamEnded?.Invoke(new VideoStreamEndedEvent
            {
                StreamId = streamId,
                VehicleId = stream.VehicleId,
                Duration = (stream.EndedAt!.Value - stream.StartedAt).TotalSeconds
            });
        }
    }
    
    /// <summary>
    /// Push video frame to stream.
    /// </summary>
    public void PushFrame(string streamId, VideoFrame frame)
    {
        if (!_activeStreams.TryGetValue(streamId, out var stream))
            return;
        
        // Encode frame
        var encoded = stream.Encoder.Encode(frame);
        
        // Update stream stats
        stream.FrameCount++;
        stream.BytesSent += encoded.Length;
        stream.LastFrameTime = DateTime.UtcNow;
        
        // Distribute to consumers
        if (_consumers.TryGetValue(streamId, out var consumers))
        {
            foreach (var consumer in consumers)
            {
                consumer.OnFrame(encoded);
            }
        }
    }
    
    /// <summary>
    /// Subscribe to a stream.
    /// </summary>
    public void SubscribeToStream(string streamId, IStreamConsumer consumer)
    {
        if (!_consumers.ContainsKey(streamId))
            _consumers[streamId] = new List<IStreamConsumer>();
        
        _consumers[streamId].Add(consumer);
    }
    
    /// <summary>
    /// Get stream info.
    /// </summary>
    public VideoStream? GetStream(string streamId)
    {
        return _activeStreams.TryGetValue(streamId, out var stream) ? stream : null;
    }
    
    /// <summary>
    /// Get all active streams.
    /// </summary>
    public IReadOnlyList<VideoStream> GetActiveStreams()
    {
        return _activeStreams.Values.ToList();
    }
    
    /// <summary>
    /// Get streaming statistics.
    /// </summary>
    public StreamingStats GetStats()
    {
        return new StreamingStats
        {
            ActiveStreamCount = _activeStreams.Count,
            TotalBandwidthMbps = _activeStreams.Values.Sum(s => s.BitrateKbps) / 1000.0,
            TotalFramesSent = _activeStreams.Values.Sum(s => s.FrameCount),
            TotalBytesSent = _activeStreams.Values.Sum(s => s.BytesSent)
        };
    }
    
    private VideoEncoder ConfigureEncoder(VideoQuality quality) => quality switch
    {
        VideoQuality.Low => new VideoEncoder { Width = 640, Height = 360, BitrateKbps = 500, Fps = 15 },
        VideoQuality.Medium => new VideoEncoder { Width = 1280, Height = 720, BitrateKbps = 2000, Fps = 30 },
        VideoQuality.High => new VideoEncoder { Width = 1920, Height = 1080, BitrateKbps = 5000, Fps = 30 },
        VideoQuality.Ultra => new VideoEncoder { Width = 3840, Height = 2160, BitrateKbps = 15000, Fps = 30 },
        _ => new VideoEncoder { Width = 1280, Height = 720, BitrateKbps = 2000, Fps = 30 }
    };
    
    public void Dispose()
    {
        foreach (var stream in _activeStreams.Values)
        {
            stream.Status = StreamStatus.Stopped;
        }
    }
}

/// <summary>
/// Geospatial services for route planning and analysis.
/// </summary>
public class GeospatialService
{
    private readonly GeospatialConfig _config;
    private readonly ElevationService _elevationService;
    private readonly WeatherService _weatherService;
    private readonly AirspaceService _airspaceService;
    
    public GeospatialService(GeospatialConfig? config = null)
    {
        _config = config ?? new GeospatialConfig();
        _elevationService = new ElevationService(_config);
        _weatherService = new WeatherService(_config);
        _airspaceService = new AirspaceService(_config);
    }
    
    /// <summary>
    /// Plan optimal route considering terrain, weather, and airspace.
    /// </summary>
    public async Task<RoutePlanResult> PlanRouteAsync(
        RoutePlanRequest request,
        CancellationToken ct = default)
    {
        var result = new RoutePlanResult { RequestId = Guid.NewGuid().ToString() };
        
        // Get elevation data along route
        var elevationProfile = await _elevationService.GetElevationProfileAsync(
            request.Waypoints,
            ct);
        
        // Check airspace constraints
        var airspaceResult = await _airspaceService.CheckRouteAsync(
            request.Waypoints,
            request.MaxAltitudeAgl,
            ct);
        
        // Get weather along route
        var weatherResult = await _weatherService.GetRouteWeatherAsync(
            request.Waypoints,
            request.DepartureTime,
            ct);
        
        // Calculate optimized waypoints
        result.OptimizedWaypoints = OptimizeWaypoints(
            request.Waypoints,
            elevationProfile,
            airspaceResult,
            request);
        
        // Calculate metrics
        result.TotalDistanceKm = CalculateRouteDistance(result.OptimizedWaypoints);
        result.EstimatedFlightTimeMinutes = result.TotalDistanceKm / (request.CruiseSpeedKmh / 60);
        result.ElevationProfile = elevationProfile;
        result.AirspaceConstraints = airspaceResult.Constraints;
        result.WeatherSummary = weatherResult;
        
        // Calculate energy requirements
        result.EstimatedBatteryUsagePercent = CalculateBatteryUsage(
            result.TotalDistanceKm,
            elevationProfile,
            request.VehicleType);
        
        result.IsFeasible = result.EstimatedBatteryUsagePercent <= 80 && // 20% reserve
                           !airspaceResult.HasBlockingRestrictions &&
                           weatherResult.IsFlyable;
        
        return result;
    }
    
    /// <summary>
    /// Calculate terrain clearance for a position.
    /// </summary>
    public async Task<TerrainClearance> GetTerrainClearanceAsync(
        double latitude,
        double longitude,
        double altitudeMsl,
        CancellationToken ct = default)
    {
        var elevation = await _elevationService.GetElevationAsync(latitude, longitude, ct);
        
        return new TerrainClearance
        {
            Latitude = latitude,
            Longitude = longitude,
            GroundElevationMsl = elevation,
            AircraftAltitudeMsl = altitudeMsl,
            ClearanceMeters = altitudeMsl - elevation,
            IsSafe = (altitudeMsl - elevation) > _config.MinTerrainClearanceMeters
        };
    }
    
    /// <summary>
    /// Find safe landing zones in an area.
    /// </summary>
    public async Task<List<LandingZone>> FindLandingZonesAsync(
        double centerLat,
        double centerLon,
        double radiusKm,
        CancellationToken ct = default)
    {
        var zones = new List<LandingZone>();
        
        // Grid search for flat areas
        int gridSize = 10;
        double stepDeg = radiusKm / 111.0 / gridSize; // Approx degrees per km
        
        for (int i = -gridSize; i <= gridSize; i++)
        {
            for (int j = -gridSize; j <= gridSize; j++)
            {
                double lat = centerLat + i * stepDeg;
                double lon = centerLon + j * stepDeg / SysMath.Cos(centerLat * SysMath.PI / 180);
                
                var slopeScore = await EvaluateLandingZoneAsync(lat, lon, ct);
                
                if (slopeScore >= 0.7)
                {
                    zones.Add(new LandingZone
                    {
                        Latitude = lat,
                        Longitude = lon,
                        SuitabilityScore = slopeScore,
                        DistanceFromCenterKm = SysMath.Sqrt(i * i + j * j) * radiusKm / gridSize
                    });
                }
            }
        }
        
        return zones.OrderByDescending(z => z.SuitabilityScore).Take(5).ToList();
    }
    
    private List<GeoWaypoint> OptimizeWaypoints(
        List<GeoWaypoint> original,
        ElevationProfile elevation,
        RouteAirspaceCheckResult airspace,
        RoutePlanRequest request)
    {
        var optimized = new List<GeoWaypoint>();
        
        for (int i = 0; i < original.Count; i++)
        {
            var wp = original[i];
            double groundElevation = elevation.Points.FirstOrDefault(p =>
                SysMath.Abs(p.Latitude - wp.Latitude) < 0.001 &&
                SysMath.Abs(p.Longitude - wp.Longitude) < 0.001)?.Elevation ?? 0;
            
            // Ensure terrain clearance
            double minAltitude = groundElevation + _config.MinTerrainClearanceMeters;
            
            // Check airspace ceiling
            var constraint = airspace.Constraints.FirstOrDefault(c =>
                c.Contains(wp.Latitude, wp.Longitude));
            double maxAltitude = constraint?.CeilingMeters ?? request.MaxAltitudeAgl + groundElevation;
            
            optimized.Add(new GeoWaypoint
            {
                Latitude = wp.Latitude,
                Longitude = wp.Longitude,
                AltitudeMsl = SysMath.Clamp(wp.AltitudeMsl, minAltitude, maxAltitude),
                Speed = wp.Speed,
                Action = wp.Action
            });
        }
        
        return optimized;
    }
    
    private double CalculateRouteDistance(List<GeoWaypoint> waypoints)
    {
        double total = 0;
        for (int i = 1; i < waypoints.Count; i++)
        {
            total += HaversineDistance(
                waypoints[i - 1].Latitude, waypoints[i - 1].Longitude,
                waypoints[i].Latitude, waypoints[i].Longitude);
        }
        return total;
    }
    
    private double HaversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        const double R = 6371; // Earth radius km
        double dLat = (lat2 - lat1) * SysMath.PI / 180;
        double dLon = (lon2 - lon1) * SysMath.PI / 180;
        double a = SysMath.Sin(dLat / 2) * SysMath.Sin(dLat / 2) +
                   SysMath.Cos(lat1 * SysMath.PI / 180) * SysMath.Cos(lat2 * SysMath.PI / 180) *
                   SysMath.Sin(dLon / 2) * SysMath.Sin(dLon / 2);
        return R * 2 * SysMath.Atan2(SysMath.Sqrt(a), SysMath.Sqrt(1 - a));
    }
    
    private double CalculateBatteryUsage(double distanceKm, ElevationProfile elevation, string vehicleType)
    {
        // Simplified model: base consumption + altitude gains
        double baseConsumption = distanceKm * 5; // 5% per km baseline
        
        double altitudeGain = 0;
        for (int i = 1; i < elevation.Points.Count; i++)
        {
            var delta = elevation.Points[i].Elevation - elevation.Points[i - 1].Elevation;
            if (delta > 0) altitudeGain += delta;
        }
        
        double climbConsumption = altitudeGain * 0.1; // 0.1% per meter climb
        
        return SysMath.Min(100, baseConsumption + climbConsumption);
    }
    
    private async Task<double> EvaluateLandingZoneAsync(double lat, double lon, CancellationToken ct)
    {
        // Get elevation for surrounding points to calculate slope
        // Simplified - would use DEM data
        return 0.8; // Placeholder
    }
}

/// <summary>
/// Elevation data service.
/// </summary>
public class ElevationService
{
    private readonly GeospatialConfig _config;
    
    public ElevationService(GeospatialConfig config)
    {
        _config = config;
    }
    
    public async Task<ElevationProfile> GetElevationProfileAsync(
        List<GeoWaypoint> waypoints,
        CancellationToken ct)
    {
        var profile = new ElevationProfile();
        
        foreach (var wp in waypoints)
        {
            var elevation = await GetElevationAsync(wp.Latitude, wp.Longitude, ct);
            profile.Points.Add(new ElevationPoint
            {
                Latitude = wp.Latitude,
                Longitude = wp.Longitude,
                Elevation = elevation
            });
        }
        
        profile.MinElevation = profile.Points.Min(p => p.Elevation);
        profile.MaxElevation = profile.Points.Max(p => p.Elevation);
        
        return profile;
    }
    
    public Task<double> GetElevationAsync(double lat, double lon, CancellationToken ct)
    {
        // In production, would call DEM API or use local data
        // Simplified - return sea level
        return Task.FromResult(0.0);
    }
}

/// <summary>
/// Weather data service.
/// </summary>
public class WeatherService
{
    private readonly GeospatialConfig _config;
    
    public WeatherService(GeospatialConfig config)
    {
        _config = config;
    }
    
    public async Task<RouteWeatherResult> GetRouteWeatherAsync(
        List<GeoWaypoint> waypoints,
        DateTime departureTime,
        CancellationToken ct)
    {
        // In production, would call weather API
        return new RouteWeatherResult
        {
            IsFlyable = true,
            WindSpeed = 5,
            WindDirection = 180,
            Visibility = 10,
            CloudCeiling = 3000,
            PrecipitationProbability = 0
        };
    }
}

/// <summary>
/// Airspace data service.
/// </summary>
public class AirspaceService
{
    private readonly GeospatialConfig _config;
    
    public AirspaceService(GeospatialConfig config)
    {
        _config = config;
    }
    
    public async Task<RouteAirspaceCheckResult> CheckRouteAsync(
        List<GeoWaypoint> waypoints,
        double maxAltitudeAgl,
        CancellationToken ct)
    {
        // In production, would check against airspace database
        return new RouteAirspaceCheckResult
        {
            HasBlockingRestrictions = false,
            Constraints = new List<RouteAirspaceConstraint>()
        };
    }
}

// Data types for event bus
public class EventBusConfig
{
    public bool EnableEventSourcing { get; set; } = true;
    public bool SendUnhandledToDeadLetter { get; set; } = true;
    public string JournalPath { get; set; } = "logs/events.jsonl";
}

public class EventEnvelope
{
    public string Id { get; set; } = "";
    public DateTime Timestamp { get; set; }
    public string EventType { get; set; } = "";
    public object Payload { get; set; } = null!;
    public string? StreamName { get; set; }
    public string CorrelationId { get; set; } = "";
    public string? CausationId { get; set; }
    public Dictionary<string, string> Metadata { get; set; } = new();
}

public class PublishOptions
{
    public string? StreamName { get; set; }
    public string? CorrelationId { get; set; }
    public string? CausationId { get; set; }
    public Dictionary<string, string>? Metadata { get; set; }
}

public class DeadLetterEvent
{
    public EventEnvelope OriginalEvent { get; set; } = null!;
    public string Reason { get; set; } = "";
    public string? Exception { get; set; }
    public DateTime Timestamp { get; set; }
    public int RetryCount { get; set; }
}

public interface IEventHandler
{
    Task HandleAsync(object payload);
}

public class TypedEventHandler<T> : IEventHandler where T : class
{
    private readonly Action<T> _handler;
    
    public TypedEventHandler(Action<T> handler)
    {
        _handler = handler;
    }
    
    public Task HandleAsync(object payload)
    {
        if (payload is T typed)
            _handler(typed);
        return Task.CompletedTask;
    }
}

public class EventStream
{
    public string Name { get; }
    public List<Action<EventEnvelope>> Handlers { get; } = new();
    
    public EventStream(string name) { Name = name; }
}

public class Subscription : IDisposable
{
    private readonly Action _unsubscribe;
    public Subscription(Action unsubscribe) { _unsubscribe = unsubscribe; }
    public void Dispose() { _unsubscribe(); }
}

// Multi-tenant types
public class TenantRegistration
{
    public string Name { get; set; } = "";
    public TenantPlan Plan { get; set; }
    public string AdminEmail { get; set; } = "";
}

public enum TenantPlan { Starter, Professional, Enterprise }

public class TenantContext
{
    public string TenantId { get; set; } = "";
    public string Name { get; set; } = "";
    public TenantPlan Plan { get; set; }
    public DateTime CreatedAt { get; set; }
    public TenantLimits Limits { get; set; } = new();
    public string ApiKey { get; set; } = "";
    public string SecretKey { get; set; } = "";
    public HashSet<string> Vehicles { get; set; } = new();
    public HashSet<string> Pilots { get; set; } = new();
    public int ActiveFlights { get; set; }
    public double MonthlyFlightHours { get; set; }
    public double StorageUsedGb { get; set; }
    public int ApiCallsThisMonth { get; set; }
}

public class TenantLimits
{
    public int MaxVehicles { get; set; }
    public int MaxPilots { get; set; }
    public int MaxConcurrentFlights { get; set; }
    public double MaxMonthlyFlightHours { get; set; }
    public double MaxStorageGb { get; set; }
    public int MaxApiCallsPerMonth { get; set; }
}

public class TenantUsageMetrics
{
    public string TenantId { get; set; } = "";
    public int VehicleCount { get; set; }
    public int VehicleLimit { get; set; }
    public int PilotCount { get; set; }
    public int PilotLimit { get; set; }
    public double MonthlyFlightHours { get; set; }
    public double FlightHoursLimit { get; set; }
    public double StorageUsedGb { get; set; }
    public double StorageLimitGb { get; set; }
    public int ApiCallsThisMonth { get; set; }
    public int ApiCallLimit { get; set; }
}

public enum TenantOperation { StartFlight, AddVehicle, AddPilot, ApiCall }

public class TenantOperationResult
{
    public bool Allowed { get; set; }
    public string Reason { get; set; } = "";
}

public class TenantIsolationPolicy { }

public class TenantCreatedEvent
{
    public string TenantId { get; set; } = "";
    public string Name { get; set; } = "";
    public TenantPlan Plan { get; set; }
}

public class VehicleAddedToTenantEvent
{
    public string TenantId { get; set; } = "";
    public string VehicleId { get; set; } = "";
}

// Video streaming types
public class StreamingConfig
{
    public string RtmpEndpoint { get; set; } = "rtmp://stream.controlworkbench.io/live";
    public string HlsEndpoint { get; set; } = "https://stream.controlworkbench.io/hls";
    public string WebRtcEndpoint { get; set; } = "wss://stream.controlworkbench.io/webrtc";
}

public class StartStreamRequest
{
    public string VehicleId { get; set; } = "";
    public VideoQuality Quality { get; set; }
    public StreamProtocol Protocol { get; set; }
}

public enum VideoQuality { Low, Medium, High, Ultra }
public enum StreamProtocol { Rtmp, Hls, WebRtc }
public enum StreamStatus { Starting, Active, Paused, Stopped, Error }

public class VideoStream
{
    public string StreamId { get; set; } = "";
    public string VehicleId { get; set; } = "";
    public VideoQuality Quality { get; set; }
    public StreamProtocol Protocol { get; set; }
    public StreamStatus Status { get; set; }
    public DateTime StartedAt { get; set; }
    public DateTime? EndedAt { get; set; }
    public string RtmpUrl { get; set; } = "";
    public string HlsUrl { get; set; } = "";
    public string WebRtcUrl { get; set; } = "";
    public VideoEncoder Encoder { get; set; } = new();
    public long FrameCount { get; set; }
    public long BytesSent { get; set; }
    public DateTime LastFrameTime { get; set; }
    public int BitrateKbps => Encoder.BitrateKbps;
}

public class VideoEncoder
{
    public int Width { get; set; }
    public int Height { get; set; }
    public int BitrateKbps { get; set; }
    public int Fps { get; set; }
    
    public byte[] Encode(VideoFrame frame)
    {
        // In production, would use actual encoder (x264, NVENC, etc.)
        return frame.Data;
    }
}

public class VideoFrame
{
    public byte[] Data { get; set; } = Array.Empty<byte>();
    public DateTime Timestamp { get; set; }
    public int Width { get; set; }
    public int Height { get; set; }
}

public interface IStreamConsumer
{
    void OnFrame(byte[] frameData);
    void OnStreamEnded();
}

public class VideoStreamStartedEvent
{
    public string StreamId { get; set; } = "";
    public string VehicleId { get; set; } = "";
}

public class VideoStreamEndedEvent
{
    public string StreamId { get; set; } = "";
    public string VehicleId { get; set; } = "";
    public double Duration { get; set; }
}

public class StreamingStats
{
    public int ActiveStreamCount { get; set; }
    public double TotalBandwidthMbps { get; set; }
    public long TotalFramesSent { get; set; }
    public long TotalBytesSent { get; set; }
}

// Geospatial types
public class GeospatialConfig
{
    public double MinTerrainClearanceMeters { get; set; } = 50;
    public string ElevationApiEndpoint { get; set; } = "";
    public string WeatherApiEndpoint { get; set; } = "";
}

public class RoutePlanRequest
{
    public List<GeoWaypoint> Waypoints { get; set; } = new();
    public DateTime DepartureTime { get; set; }
    public double MaxAltitudeAgl { get; set; }
    public double CruiseSpeedKmh { get; set; }
    public string VehicleType { get; set; } = "";
}

public class RoutePlanResult
{
    public string RequestId { get; set; } = "";
    public bool IsFeasible { get; set; }
    public List<GeoWaypoint> OptimizedWaypoints { get; set; } = new();
    public double TotalDistanceKm { get; set; }
    public double EstimatedFlightTimeMinutes { get; set; }
    public double EstimatedBatteryUsagePercent { get; set; }
    public ElevationProfile ElevationProfile { get; set; } = new();
    public List<RouteAirspaceConstraint> AirspaceConstraints { get; set; } = new();
    public RouteWeatherResult WeatherSummary { get; set; } = new();
}

public class GeoWaypoint
{
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double AltitudeMsl { get; set; }
    public double Speed { get; set; }
    public string? Action { get; set; }
}

public class ElevationProfile
{
    public List<ElevationPoint> Points { get; set; } = new();
    public double MinElevation { get; set; }
    public double MaxElevation { get; set; }
}

public class ElevationPoint
{
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double Elevation { get; set; }
}

public class RouteAirspaceCheckResult
{
    public bool HasBlockingRestrictions { get; set; }
    public List<RouteAirspaceConstraint> Constraints { get; set; } = new();
}

public class RouteAirspaceConstraint
{
    public string Name { get; set; } = "";
    public string Type { get; set; } = "";
    public double FloorMeters { get; set; }
    public double CeilingMeters { get; set; }
    public List<(double Lat, double Lon)> Boundary { get; set; } = new();
    
    public bool Contains(double lat, double lon) => false; // Simplified
}

public class RouteWeatherResult
{
    public bool IsFlyable { get; set; }
    public double WindSpeed { get; set; }
    public double WindDirection { get; set; }
    public double Visibility { get; set; }
    public double CloudCeiling { get; set; }
    public double PrecipitationProbability { get; set; }
}

public class TerrainClearance
{
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double GroundElevationMsl { get; set; }
    public double AircraftAltitudeMsl { get; set; }
    public double ClearanceMeters { get; set; }
    public bool IsSafe { get; set; }
}

public class LandingZone
{
    public double Latitude { get; set; }
    public double Longitude { get; set; }
    public double SuitabilityScore { get; set; }
    public double DistanceFromCenterKm { get; set; }
}
