using ControlWorkbench.Core.Time;
using ControlWorkbench.Protocol;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.Transport;

/// <summary>
/// Mock transport that generates realistic telemetry data for testing.
/// </summary>
public sealed class MockTransport : ITransport
{
    private CancellationTokenSource? _cts;
    private Task? _generateTask;
    private ConnectionState _state = ConnectionState.Disconnected;
    private readonly Random _random = new();

    // Simulation state
    private double _time;
    private double _roll, _pitch, _yaw;
    private double _p, _q, _r;
    private double _x, _y;
    private double _vx, _vy;
    private double _latitude = 47.6062;
    private double _longitude = -122.3321;

    /// <summary>
    /// Gets or sets the update rate in Hz.
    /// </summary>
    public double UpdateRateHz { get; set; } = 50.0;

    /// <summary>
    /// Gets or sets whether to generate IMU messages.
    /// </summary>
    public bool GenerateImu { get; set; } = true;

    /// <summary>
    /// Gets or sets whether to generate attitude messages.
    /// </summary>
    public bool GenerateAttitude { get; set; } = true;

    /// <summary>
    /// Gets or sets whether to generate GPS messages.
    /// </summary>
    public bool GenerateGps { get; set; } = true;

    /// <summary>
    /// Gets or sets whether to generate state estimate messages.
    /// </summary>
    public bool GenerateStateEstimate { get; set; } = true;

    /// <inheritdoc/>
    public ConnectionState State
    {
        get => _state;
        private set
        {
            if (_state != value)
            {
                var old = _state;
                _state = value;
                ConnectionStateChanged?.Invoke(this, new ConnectionStateChangedEventArgs(old, value));
            }
        }
    }

    /// <inheritdoc/>
    public TransportStatistics Statistics { get; } = new();

    /// <inheritdoc/>
    public event EventHandler<MessageReceivedEventArgs>? MessageReceived;

    /// <inheritdoc/>
    public event EventHandler<ConnectionStateChangedEventArgs>? ConnectionStateChanged;

    /// <inheritdoc/>
    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        if (State == ConnectionState.Connected)
            throw new InvalidOperationException("Already connected.");

        State = ConnectionState.Connecting;
        Statistics.Reset();

        _time = 0;
        _roll = _pitch = _yaw = 0;
        _p = _q = _r = 0;
        _x = _y = 0;
        _vx = _vy = 0;

        _cts = new CancellationTokenSource();
        _generateTask = Task.Run(() => GenerateLoop(_cts.Token), _cts.Token);

        State = ConnectionState.Connected;
        return Task.CompletedTask;
    }

    /// <inheritdoc/>
    public async Task DisconnectAsync()
    {
        if (State == ConnectionState.Disconnected)
            return;

        _cts?.Cancel();

        if (_generateTask != null)
        {
            try
            {
                await _generateTask.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                // Expected
            }
        }

        _cts?.Dispose();
        _cts = null;
        _generateTask = null;

        State = ConnectionState.Disconnected;
    }

    /// <inheritdoc/>
    public Task SendAsync(IMessage message, CancellationToken cancellationToken = default)
    {
        // Mock transport ignores sent messages
        Statistics.BytesSent += MessageEncoder.Encode(message).Length;
        Statistics.PacketsSent++;
        Statistics.LastSendTime = DateTime.UtcNow;
        return Task.CompletedTask;
    }

    private async Task GenerateLoop(CancellationToken cancellationToken)
    {
        int delayMs = (int)(1000.0 / UpdateRateHz);
        double dt = 1.0 / UpdateRateHz;
        int gpsCounter = 0;
        int heartbeatCounter = 0;

        while (!cancellationToken.IsCancellationRequested)
        {
            try
            {
                await Task.Delay(delayMs, cancellationToken).ConfigureAwait(false);
                
                _time += dt;
                long timeUs = (long)(_time * 1_000_000);
                long arrivalTime = HighResolutionTime.Now.Microseconds;

                // Simulate some dynamics with noise
                double targetYaw = Math.Sin(_time * 0.2) * 0.5;
                double targetPitch = Math.Sin(_time * 0.3) * 0.1;
                double targetRoll = Math.Sin(_time * 0.25) * 0.15;

                _r = (targetYaw - _yaw) * 2.0 + GaussianNoise(0.01);
                _q = (targetPitch - _pitch) * 2.0 + GaussianNoise(0.01);
                _p = (targetRoll - _roll) * 2.0 + GaussianNoise(0.01);

                _yaw += _r * dt;
                _pitch += _q * dt;
                _roll += _p * dt;

                // Simulate 2D motion
                double speed = 2.0 + Math.Sin(_time * 0.1);
                _vx = speed * Math.Cos(_yaw) + GaussianNoise(0.1);
                _vy = speed * Math.Sin(_yaw) + GaussianNoise(0.1);
                _x += _vx * dt;
                _y += _vy * dt;

                // Generate messages

                // Heartbeat every 1 second
                if (++heartbeatCounter >= UpdateRateHz)
                {
                    heartbeatCounter = 0;
                    var heartbeat = new HeartbeatMessage
                    {
                        UptimeMs = (uint)(_time * 1000),
                        SystemState = 1,
                        Armed = 1
                    };
                    EmitMessage(heartbeat, arrivalTime);
                }

                if (GenerateImu)
                {
                    var imu = new ImuRawMessage
                    {
                        TimeUs = timeUs,
                        GyroX = (float)(_p + GaussianNoise(0.002)),
                        GyroY = (float)(_q + GaussianNoise(0.002)),
                        GyroZ = (float)(_r + GaussianNoise(0.002)),
                        AccelX = (float)(GaussianNoise(0.05)),
                        AccelY = (float)(GaussianNoise(0.05)),
                        AccelZ = (float)(9.81 + GaussianNoise(0.05)),
                        Temperature = (float)(25.0 + GaussianNoise(0.1))
                    };
                    EmitMessage(imu, arrivalTime);
                }

                if (GenerateAttitude)
                {
                    var attitude = new AttitudeMessage
                    {
                        TimeUs = timeUs,
                        Roll = (float)_roll,
                        Pitch = (float)_pitch,
                        Yaw = (float)_yaw,
                        P = (float)_p,
                        Q = (float)_q,
                        R = (float)_r
                    };
                    EmitMessage(attitude, arrivalTime);
                }

                // GPS at 10 Hz
                if (GenerateGps && ++gpsCounter >= (UpdateRateHz / 10))
                {
                    gpsCounter = 0;
                    double metersPerDegreeLat = 111000;
                    double metersPerDegreeLon = metersPerDegreeLat * Math.Cos(_latitude * Math.PI / 180);

                    var gps = new GpsMessage
                    {
                        TimeUs = timeUs,
                        Latitude = _latitude + _y / metersPerDegreeLat + GaussianNoise(0.000001),
                        Longitude = _longitude + _x / metersPerDegreeLon + GaussianNoise(0.000001),
                        Altitude = (float)(100 + GaussianNoise(0.5)),
                        VelocityNorth = (float)(_vy + GaussianNoise(0.1)),
                        VelocityEast = (float)(_vx + GaussianNoise(0.1)),
                        VelocityDown = (float)GaussianNoise(0.05),
                        HorizontalAccuracy = (float)(1.5 + Math.Abs(GaussianNoise(0.3))),
                        VerticalAccuracy = (float)(2.0 + Math.Abs(GaussianNoise(0.5)))
                    };
                    EmitMessage(gps, arrivalTime);
                }

                if (GenerateStateEstimate)
                {
                    var state = new StateEstimate2DMessage
                    {
                        TimeUs = timeUs,
                        X = (float)_x,
                        Y = (float)_y,
                        Yaw = (float)_yaw,
                        VelocityX = (float)_vx,
                        VelocityY = (float)_vy,
                        CovarianceXX = 0.01f,
                        CovarianceXY = 0.001f,
                        CovarianceYY = 0.01f,
                        CovarianceYawYaw = 0.001f
                    };
                    EmitMessage(state, arrivalTime);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
    }

    private void EmitMessage(IMessage message, long arrivalTime)
    {
        byte[] encoded = MessageEncoder.Encode(message);
        Statistics.BytesReceived += encoded.Length;
        Statistics.PacketsReceived++;
        Statistics.LastReceiveTime = DateTime.UtcNow;
        MessageReceived?.Invoke(this, new MessageReceivedEventArgs(message, arrivalTime));
    }

    private double GaussianNoise(double stdDev)
    {
        double u1 = 1.0 - _random.NextDouble();
        double u2 = 1.0 - _random.NextDouble();
        double normal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
        return normal * stdDev;
    }

    public void Dispose()
    {
        DisconnectAsync().GetAwaiter().GetResult();
    }
}
