using System.Collections.ObjectModel;
using System.IO.Ports;
using System.Windows.Input;
using System.Windows.Threading;
using ControlWorkbench.Core.Collections;
using ControlWorkbench.Protocol.Messages;
using ControlWorkbench.Transport;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// ViewModel for the Connection tab.
/// </summary>
public class ConnectionViewModel : ViewModelBase
{
    private ITransport? _transport;
    private readonly DispatcherTimer _statsTimer;
    private readonly MessageQueue<MessageReceivedEventArgs> _messageQueue;

    // Connection type
    private bool _isSerialSelected = true;
    private bool _isUdpSelected;
    private bool _isMockSelected;

    // Serial settings
    private string _selectedComPort = "";
    private int _selectedBaudRate = 115200;
    private ObservableCollection<string> _availableComPorts = new();
    private readonly int[] _baudRates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

    // UDP settings
    private int _localPort = 14550;
    private string _remoteHost = "127.0.0.1";
    private int _remotePort = 14551;

    // Mock settings
    private double _mockUpdateRateHz = 50;

    // Connection state
    private ConnectionState _connectionState = ConnectionState.Disconnected;
    private string _statusMessage = "Disconnected";

    // Statistics
    private long _bytesReceived;
    private long _packetsReceived;
    private double _bytesPerSecond;
    private double _packetsPerSecond;
    private long _crcErrors;
    private long _framingErrors;

    private long _lastBytesReceived;
    private long _lastPacketsReceived;
    private DateTime _lastStatsUpdate = DateTime.UtcNow;

    public ConnectionViewModel()
    {
        _messageQueue = new MessageQueue<MessageReceivedEventArgs>();
        
        ConnectCommand = new AsyncRelayCommand(ConnectAsync, () => ConnectionState == ConnectionState.Disconnected);
        DisconnectCommand = new AsyncRelayCommand(DisconnectAsync, () => ConnectionState == ConnectionState.Connected);
        RefreshPortsCommand = new RelayCommand(RefreshPorts);

        RefreshPorts();

        _statsTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(500)
        };
        _statsTimer.Tick += StatsTimer_Tick;
        _statsTimer.Start();
    }

    // Events for notifying other ViewModels
    public event EventHandler<IMessage>? MessageReceived;

    // Commands
    public ICommand ConnectCommand { get; }
    public ICommand DisconnectCommand { get; }
    public ICommand RefreshPortsCommand { get; }

    // Connection Type Properties
    public bool IsSerialSelected
    {
        get => _isSerialSelected;
        set
        {
            if (SetProperty(ref _isSerialSelected, value) && value)
            {
                IsUdpSelected = false;
                IsMockSelected = false;
            }
        }
    }

    public bool IsUdpSelected
    {
        get => _isUdpSelected;
        set
        {
            if (SetProperty(ref _isUdpSelected, value) && value)
            {
                IsSerialSelected = false;
                IsMockSelected = false;
            }
        }
    }

    public bool IsMockSelected
    {
        get => _isMockSelected;
        set
        {
            if (SetProperty(ref _isMockSelected, value) && value)
            {
                IsSerialSelected = false;
                IsUdpSelected = false;
            }
        }
    }

    // Serial Properties
    public ObservableCollection<string> AvailableComPorts
    {
        get => _availableComPorts;
        set => SetProperty(ref _availableComPorts, value);
    }

    public string SelectedComPort
    {
        get => _selectedComPort;
        set => SetProperty(ref _selectedComPort, value);
    }

    public int[] BaudRates => _baudRates;

    public int SelectedBaudRate
    {
        get => _selectedBaudRate;
        set => SetProperty(ref _selectedBaudRate, value);
    }

    // UDP Properties
    public int LocalPort
    {
        get => _localPort;
        set => SetProperty(ref _localPort, value);
    }

    public string RemoteHost
    {
        get => _remoteHost;
        set => SetProperty(ref _remoteHost, value);
    }

    public int RemotePort
    {
        get => _remotePort;
        set => SetProperty(ref _remotePort, value);
    }

    // Mock Properties
    public double MockUpdateRateHz
    {
        get => _mockUpdateRateHz;
        set => SetProperty(ref _mockUpdateRateHz, value);
    }

    // Connection State Properties
    public ConnectionState ConnectionState
    {
        get => _connectionState;
        private set
        {
            SetProperty(ref _connectionState, value);
            OnPropertyChanged(nameof(IsConnected));
            OnPropertyChanged(nameof(CanConnect));
        }
    }

    public bool IsConnected => ConnectionState == ConnectionState.Connected;
    public bool CanConnect => ConnectionState == ConnectionState.Disconnected;

    public string StatusMessage
    {
        get => _statusMessage;
        private set => SetProperty(ref _statusMessage, value);
    }

    // Statistics Properties
    public long BytesReceived
    {
        get => _bytesReceived;
        private set => SetProperty(ref _bytesReceived, value);
    }

    public long PacketsReceived
    {
        get => _packetsReceived;
        private set => SetProperty(ref _packetsReceived, value);
    }

    public double BytesPerSecond
    {
        get => _bytesPerSecond;
        private set => SetProperty(ref _bytesPerSecond, value);
    }

    public double PacketsPerSecond
    {
        get => _packetsPerSecond;
        private set => SetProperty(ref _packetsPerSecond, value);
    }

    public long CrcErrors
    {
        get => _crcErrors;
        private set => SetProperty(ref _crcErrors, value);
    }

    public long FramingErrors
    {
        get => _framingErrors;
        private set => SetProperty(ref _framingErrors, value);
    }

    private void RefreshPorts()
    {
        AvailableComPorts.Clear();
        foreach (var port in SerialPort.GetPortNames())
        {
            AvailableComPorts.Add(port);
        }
        if (AvailableComPorts.Count > 0 && string.IsNullOrEmpty(SelectedComPort))
        {
            SelectedComPort = AvailableComPorts[0];
        }
    }

    private async Task ConnectAsync()
    {
        try
        {
            StatusMessage = "Connecting...";

            if (IsSerialSelected)
            {
                var serial = new SerialTransport
                {
                    PortName = SelectedComPort,
                    BaudRate = SelectedBaudRate
                };
                _transport = serial;
            }
            else if (IsUdpSelected)
            {
                var udp = new UdpTransport
                {
                    LocalPort = LocalPort,
                    RemoteHost = RemoteHost,
                    RemotePort = RemotePort
                };
                _transport = udp;
            }
            else if (IsMockSelected)
            {
                var mock = new MockTransport
                {
                    UpdateRateHz = MockUpdateRateHz
                };
                _transport = mock;
            }

            if (_transport != null)
            {
                _transport.MessageReceived += Transport_MessageReceived;
                _transport.ConnectionStateChanged += Transport_ConnectionStateChanged;
                await _transport.ConnectAsync();
            }
        }
        catch (Exception ex)
        {
            StatusMessage = $"Connection failed: {ex.Message}";
            ConnectionState = ConnectionState.Error;
        }
    }

    private async Task DisconnectAsync()
    {
        if (_transport != null)
        {
            _transport.MessageReceived -= Transport_MessageReceived;
            _transport.ConnectionStateChanged -= Transport_ConnectionStateChanged;
            await _transport.DisconnectAsync();
            _transport.Dispose();
            _transport = null;
        }
        StatusMessage = "Disconnected";
        ConnectionState = ConnectionState.Disconnected;
    }

    private void Transport_MessageReceived(object? sender, MessageReceivedEventArgs e)
    {
        _messageQueue.TryWrite(e);
        MessageReceived?.Invoke(this, e.Message);
    }

    private void Transport_ConnectionStateChanged(object? sender, ConnectionStateChangedEventArgs e)
    {
        System.Windows.Application.Current?.Dispatcher.Invoke(() =>
        {
            ConnectionState = e.NewState;
            StatusMessage = e.NewState switch
            {
                ConnectionState.Connected => "Connected",
                ConnectionState.Connecting => "Connecting...",
                ConnectionState.Disconnected => "Disconnected",
                ConnectionState.Error => $"Error: {e.Message}",
                ConnectionState.Reconnecting => "Reconnecting...",
                _ => e.NewState.ToString()
            };
        });
    }

    private void StatsTimer_Tick(object? sender, EventArgs e)
    {
        if (_transport != null)
        {
            var stats = _transport.Statistics;
            var now = DateTime.UtcNow;
            var elapsed = (now - _lastStatsUpdate).TotalSeconds;

            if (elapsed > 0)
            {
                BytesPerSecond = (stats.BytesReceived - _lastBytesReceived) / elapsed;
                PacketsPerSecond = (stats.PacketsReceived - _lastPacketsReceived) / elapsed;
            }

            BytesReceived = stats.BytesReceived;
            PacketsReceived = stats.PacketsReceived;

            _lastBytesReceived = stats.BytesReceived;
            _lastPacketsReceived = stats.PacketsReceived;
            _lastStatsUpdate = now;
        }
    }

    public async Task SendMessageAsync(IMessage message)
    {
        if (_transport != null && ConnectionState == ConnectionState.Connected)
        {
            await _transport.SendAsync(message);
        }
    }
}
