using System.Collections.ObjectModel;
using System.Windows.Input;
using ControlWorkbench.Protocol.Messages;

namespace ControlWorkbench.App.ViewModels;

/// <summary>
/// Represents a device parameter.
/// </summary>
public class ParameterItem : ViewModelBase
{
    private string _name = "";
    private float _currentValue;
    private float _desiredValue;
    private DateTime _lastReceivedTime;
    private bool _isModified;

    public string Name
    {
        get => _name;
        set => SetProperty(ref _name, value);
    }

    public float CurrentValue
    {
        get => _currentValue;
        set
        {
            if (SetProperty(ref _currentValue, value))
            {
                LastReceivedTime = DateTime.Now;
                UpdateModifiedState();
            }
        }
    }

    public float DesiredValue
    {
        get => _desiredValue;
        set
        {
            if (SetProperty(ref _desiredValue, value))
            {
                UpdateModifiedState();
            }
        }
    }

    public DateTime LastReceivedTime
    {
        get => _lastReceivedTime;
        set => SetProperty(ref _lastReceivedTime, value);
    }

    public bool IsModified
    {
        get => _isModified;
        private set => SetProperty(ref _isModified, value);
    }

    private void UpdateModifiedState()
    {
        IsModified = System.Math.Abs(CurrentValue - DesiredValue) > 1e-6f;
    }

    public void SyncDesiredToCurrent()
    {
        DesiredValue = CurrentValue;
    }
}

/// <summary>
/// ViewModel for the Parameter Editor tab.
/// </summary>
public class ParameterEditorViewModel : ViewModelBase
{
    private readonly Func<IMessage, Task>? _sendMessageFunc;
    private ParameterItem? _selectedParameter;
    private string _newParameterName = "";
    private float _newParameterValue;

    public ParameterEditorViewModel(Func<IMessage, Task>? sendMessageFunc = null)
    {
        _sendMessageFunc = sendMessageFunc;

        Parameters = new ObservableCollection<ParameterItem>();

        WriteSelectedCommand = new AsyncRelayCommand(WriteSelectedAsync, () => SelectedParameter != null);
        WriteAllChangedCommand = new AsyncRelayCommand(WriteAllChangedAsync, () => Parameters.Any(p => p.IsModified));
        RefreshParametersCommand = new RelayCommand(RefreshParameters);
        AddParameterCommand = new RelayCommand(AddParameter, () => !string.IsNullOrWhiteSpace(NewParameterName));
        RevertSelectedCommand = new RelayCommand(RevertSelected, () => SelectedParameter != null);
        RevertAllCommand = new RelayCommand(RevertAll);

        // Add some default parameters for demonstration
        AddDefaultParameters();
    }

    public ObservableCollection<ParameterItem> Parameters { get; }

    public ParameterItem? SelectedParameter
    {
        get => _selectedParameter;
        set => SetProperty(ref _selectedParameter, value);
    }

    public string NewParameterName
    {
        get => _newParameterName;
        set => SetProperty(ref _newParameterName, value);
    }

    public float NewParameterValue
    {
        get => _newParameterValue;
        set => SetProperty(ref _newParameterValue, value);
    }

    public int ModifiedCount => Parameters.Count(p => p.IsModified);

    public ICommand WriteSelectedCommand { get; }
    public ICommand WriteAllChangedCommand { get; }
    public ICommand RefreshParametersCommand { get; }
    public ICommand AddParameterCommand { get; }
    public ICommand RevertSelectedCommand { get; }
    public ICommand RevertAllCommand { get; }

    private void AddDefaultParameters()
    {
        var defaults = new (string Name, float Value)[]
        {
            ("PID_RATE_P", 0.15f),
            ("PID_RATE_I", 0.05f),
            ("PID_RATE_D", 0.01f),
            ("PID_ANGLE_P", 4.5f),
            ("PID_ANGLE_I", 0.5f),
            ("PID_VEL_P", 1.0f),
            ("PID_VEL_I", 0.2f),
            ("PID_POS_P", 0.5f),
            ("MAX_RATE", 200.0f),
            ("MAX_ANGLE", 45.0f),
            ("MAX_VELOCITY", 5.0f),
            ("COMP_FILTER_ALPHA", 0.98f),
            ("GYRO_BIAS_X", 0.0f),
            ("GYRO_BIAS_Y", 0.0f),
            ("GYRO_BIAS_Z", 0.0f),
        };

        foreach (var (name, value) in defaults)
        {
            var param = new ParameterItem
            {
                Name = name,
                CurrentValue = value,
                DesiredValue = value,
                LastReceivedTime = DateTime.Now
            };
            Parameters.Add(param);
        }
    }

    public void ProcessParamValue(ParamValueMessage msg)
    {
        var existing = Parameters.FirstOrDefault(p => p.Name == msg.Name);
        if (existing != null)
        {
            existing.CurrentValue = msg.Value;
        }
        else
        {
            var param = new ParameterItem
            {
                Name = msg.Name,
                CurrentValue = msg.Value,
                DesiredValue = msg.Value,
                LastReceivedTime = DateTime.Now
            };
            Parameters.Add(param);
        }
        OnPropertyChanged(nameof(ModifiedCount));
    }

    private async Task WriteSelectedAsync()
    {
        if (SelectedParameter == null || _sendMessageFunc == null) return;

        var msg = new ParamSetMessage
        {
            Name = SelectedParameter.Name,
            Value = SelectedParameter.DesiredValue,
            ParamType = 0
        };

        await _sendMessageFunc(msg);
        SelectedParameter.CurrentValue = SelectedParameter.DesiredValue;
        OnPropertyChanged(nameof(ModifiedCount));
    }

    private async Task WriteAllChangedAsync()
    {
        if (_sendMessageFunc == null) return;

        foreach (var param in Parameters.Where(p => p.IsModified))
        {
            var msg = new ParamSetMessage
            {
                Name = param.Name,
                Value = param.DesiredValue,
                ParamType = 0
            };

            await _sendMessageFunc(msg);
            param.CurrentValue = param.DesiredValue;
        }
        OnPropertyChanged(nameof(ModifiedCount));
    }

    private void RefreshParameters()
    {
        // In a real implementation, this would request all parameters from the device
        // For now, just reset the timestamps
        foreach (var param in Parameters)
        {
            param.LastReceivedTime = DateTime.Now;
        }
    }

    private void AddParameter()
    {
        if (string.IsNullOrWhiteSpace(NewParameterName)) return;
        if (Parameters.Any(p => p.Name == NewParameterName)) return;

        var param = new ParameterItem
        {
            Name = NewParameterName,
            CurrentValue = NewParameterValue,
            DesiredValue = NewParameterValue,
            LastReceivedTime = DateTime.Now
        };
        Parameters.Add(param);

        NewParameterName = "";
        NewParameterValue = 0;
    }

    private void RevertSelected()
    {
        SelectedParameter?.SyncDesiredToCurrent();
        OnPropertyChanged(nameof(ModifiedCount));
    }

    private void RevertAll()
    {
        foreach (var param in Parameters)
        {
            param.SyncDesiredToCurrent();
        }
        OnPropertyChanged(nameof(ModifiedCount));
    }
}
