using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// Main ViewModel that coordinates all tab ViewModels.
/// </summary>
public class MainViewModel : ViewModelBase
{
    public MainViewModel()
    {
        Connection = new ConnectionViewModel();
        TelemetryDashboard = new TelemetryDashboardViewModel();
        ParameterEditor = new ParameterEditorViewModel(msg => Connection.SendMessageAsync(msg));
        MathWorkbench = new MathWorkbenchViewModel();
        ControlDesign = new ControlDesignViewModel();
        Tools = new ToolsViewModel();
        Logs = new LogsViewModel();

        // Wire up message routing
        Connection.MessageReceived += OnMessageReceived;
    }

    public ConnectionViewModel Connection { get; }
    public TelemetryDashboardViewModel TelemetryDashboard { get; }
    public ParameterEditorViewModel ParameterEditor { get; }
    public MathWorkbenchViewModel MathWorkbench { get; }
    public ControlDesignViewModel ControlDesign { get; }
    public ToolsViewModel Tools { get; }
    public LogsViewModel Logs { get; }

    private void OnMessageReceived(object? sender, IMessage message)
    {
        System.Windows.Application.Current?.Dispatcher.BeginInvoke(() =>
        {
            TelemetryDashboard.ProcessMessage(message);

            if (message is ParamValueMessage pvm)
            {
                ParameterEditor.ProcessParamValue(pvm);
            }
        });
    }
}
