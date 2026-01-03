namespace ControlWorkbench.VEX;

/// <summary>
/// Tunable parameter that can be adjusted remotely from ControlWorkbench.
/// </summary>
public class TunableParameter
{
    public string Name { get; }
    public string Category { get; }
    public double Value { get; set; }
    public double MinValue { get; }
    public double MaxValue { get; }
    public double DefaultValue { get; }
    public double Step { get; }
    public string Unit { get; }
    public string Description { get; }

    public event Action<double>? ValueChanged;

    public TunableParameter(
        string name,
        double defaultValue,
        double minValue = double.MinValue,
        double maxValue = double.MaxValue,
        double step = 0.01,
        string category = "General",
        string unit = "",
        string description = "")
    {
        Name = name;
        Value = defaultValue;
        DefaultValue = defaultValue;
        MinValue = minValue;
        MaxValue = maxValue;
        Step = step;
        Category = category;
        Unit = unit;
        Description = description;
    }

    public void SetValue(double value)
    {
        value = System.Math.Clamp(value, MinValue, MaxValue);
        if (System.Math.Abs(value - Value) > double.Epsilon)
        {
            Value = value;
            ValueChanged?.Invoke(value);
        }
    }

    public void Reset() => SetValue(DefaultValue);
}

/// <summary>
/// Manager for remote parameter tuning.
/// </summary>
public class ParameterManager
{
    private readonly Dictionary<string, TunableParameter> _parameters = new();
    private readonly List<string> _orderedKeys = new();

    public IReadOnlyDictionary<string, TunableParameter> Parameters => _parameters;

    public event Action<TunableParameter>? ParameterAdded;
    public event Action<string, double>? ParameterValueChanged;

    /// <summary>
    /// Register a tunable parameter.
    /// </summary>
    public TunableParameter Register(
        string name,
        double defaultValue,
        double minValue = double.MinValue,
        double maxValue = double.MaxValue,
        double step = 0.01,
        string category = "General",
        string unit = "",
        string description = "")
    {
        if (_parameters.ContainsKey(name))
            return _parameters[name];

        var param = new TunableParameter(name, defaultValue, minValue, maxValue, step, category, unit, description);
        param.ValueChanged += v => ParameterValueChanged?.Invoke(name, v);
        
        _parameters[name] = param;
        _orderedKeys.Add(name);
        ParameterAdded?.Invoke(param);

        return param;
    }

    /// <summary>
    /// Register a PID controller's parameters.
    /// </summary>
    public (TunableParameter kp, TunableParameter ki, TunableParameter kd) RegisterPid(
        string prefix,
        double kp = 1.0, double ki = 0.0, double kd = 0.0,
        double maxGain = 100.0)
    {
        var kpParam = Register($"{prefix}.kP", kp, 0, maxGain, 0.01, prefix, "", "Proportional gain");
        var kiParam = Register($"{prefix}.kI", ki, 0, maxGain, 0.001, prefix, "", "Integral gain");
        var kdParam = Register($"{prefix}.kD", kd, 0, maxGain, 0.001, prefix, "", "Derivative gain");
        
        return (kpParam, kiParam, kdParam);
    }

    /// <summary>
    /// Get parameter value by name.
    /// </summary>
    public double Get(string name) => _parameters.TryGetValue(name, out var p) ? p.Value : 0;

    /// <summary>
    /// Set parameter value by name.
    /// </summary>
    public void Set(string name, double value)
    {
        if (_parameters.TryGetValue(name, out var p))
            p.SetValue(value);
    }

    /// <summary>
    /// Get all parameters in a category.
    /// </summary>
    public IEnumerable<TunableParameter> GetByCategory(string category) =>
        _parameters.Values.Where(p => p.Category == category);

    /// <summary>
    /// Get all category names.
    /// </summary>
    public IEnumerable<string> GetCategories() =>
        _parameters.Values.Select(p => p.Category).Distinct();

    /// <summary>
    /// Reset all parameters to defaults.
    /// </summary>
    public void ResetAll()
    {
        foreach (var p in _parameters.Values)
            p.Reset();
    }

    /// <summary>
    /// Export parameters to JSON format.
    /// </summary>
    public string ExportJson()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("{");
        
        bool first = true;
        foreach (var key in _orderedKeys)
        {
            if (!first) sb.AppendLine(",");
            first = false;
            var p = _parameters[key];
            sb.Append($"  \"{key}\": {p.Value:G10}");
        }
        
        sb.AppendLine();
        sb.AppendLine("}");
        return sb.ToString();
    }

    /// <summary>
    /// Import parameters from JSON format.
    /// </summary>
    public void ImportJson(string json)
    {
        // Simple JSON parsing for parameter values
        var lines = json.Split('\n');
        foreach (var line in lines)
        {
            var trimmed = line.Trim().TrimEnd(',');
            if (trimmed.StartsWith("\"") && trimmed.Contains(":"))
            {
                var parts = trimmed.Split(':');
                if (parts.Length == 2)
                {
                    var name = parts[0].Trim().Trim('"');
                    if (double.TryParse(parts[1].Trim(), out double value))
                    {
                        Set(name, value);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Export parameters to PROS C++ header format.
    /// </summary>
    public string ExportProsCpp()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("// Auto-generated by ControlWorkbench");
        sb.AppendLine("// Do not edit manually - changes will be overwritten");
        sb.AppendLine();
        sb.AppendLine("#pragma once");
        sb.AppendLine();
        sb.AppendLine("namespace tuning {");
        sb.AppendLine();

        var categories = GetCategories().ToList();
        foreach (var category in categories)
        {
            sb.AppendLine($"namespace {SanitizeName(category)} {{");
            foreach (var p in GetByCategory(category))
            {
                var varName = SanitizeName(p.Name.Replace($"{category}.", ""));
                sb.AppendLine($"    constexpr double {varName} = {p.Value:G10};");
            }
            sb.AppendLine("}");
            sb.AppendLine();
        }

        sb.AppendLine("} // namespace tuning");
        return sb.ToString();
    }

    private string SanitizeName(string name) =>
        name.Replace(".", "_").Replace("-", "_").Replace(" ", "_");
}

/// <summary>
/// PID controller with integrated remote tuning support.
/// </summary>
public class TunablePidController
{
    private readonly TunableParameter _kp;
    private readonly TunableParameter _ki;
    private readonly TunableParameter _kd;
    
    private double _integral;
    private double _previousError;
    private double _previousMeasurement;
    private double _outputMin = double.MinValue;
    private double _outputMax = double.MaxValue;
    private bool _useDerivativeOnMeasurement = true;

    public double Kp => _kp.Value;
    public double Ki => _ki.Value;
    public double Kd => _kd.Value;

    public TunablePidController(ParameterManager manager, string name, 
        double kp = 1.0, double ki = 0.0, double kd = 0.0)
    {
        var gains = manager.RegisterPid(name, kp, ki, kd);
        _kp = gains.kp;
        _ki = gains.ki;
        _kd = gains.kd;
    }

    public double Compute(double setpoint, double measurement, double dt)
    {
        double error = setpoint - measurement;

        // Proportional
        double pTerm = _kp.Value * error;

        // Integral with anti-windup
        _integral += error * dt;
        _integral = System.Math.Clamp(_integral, _outputMin / (_ki.Value + 0.001), 
                                                  _outputMax / (_ki.Value + 0.001));
        double iTerm = _ki.Value * _integral;

        // Derivative
        double derivative;
        if (_useDerivativeOnMeasurement)
        {
            derivative = -(measurement - _previousMeasurement) / dt;
            _previousMeasurement = measurement;
        }
        else
        {
            derivative = (error - _previousError) / dt;
            _previousError = error;
        }
        double dTerm = _kd.Value * derivative;

        double output = pTerm + iTerm + dTerm;
        return System.Math.Clamp(output, _outputMin, _outputMax);
    }

    public void SetOutputLimits(double min, double max)
    {
        _outputMin = min;
        _outputMax = max;
    }

    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _previousMeasurement = 0;
    }
}

/// <summary>
/// Configuration preset for saving/loading robot configurations.
/// </summary>
public class ConfigurationPreset
{
    public string Name { get; set; } = "Default";
    public string Description { get; set; } = "";
    public DateTime Created { get; set; } = DateTime.Now;
    public DateTime Modified { get; set; } = DateTime.Now;
    public Dictionary<string, double> Parameters { get; set; } = new();
    public Dictionary<string, string> Metadata { get; set; } = new();

    public void SaveFrom(ParameterManager manager)
    {
        Parameters.Clear();
        foreach (var p in manager.Parameters)
        {
            Parameters[p.Key] = p.Value.Value;
        }
        Modified = DateTime.Now;
    }

    public void ApplyTo(ParameterManager manager)
    {
        foreach (var kvp in Parameters)
        {
            manager.Set(kvp.Key, kvp.Value);
        }
    }

    public string ToJson()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("{");
        sb.AppendLine($"  \"name\": \"{Name}\",");
        sb.AppendLine($"  \"description\": \"{Description}\",");
        sb.AppendLine($"  \"created\": \"{Created:O}\",");
        sb.AppendLine($"  \"modified\": \"{Modified:O}\",");
        sb.AppendLine("  \"parameters\": {");
        
        bool first = true;
        foreach (var kvp in Parameters)
        {
            if (!first) sb.AppendLine(",");
            first = false;
            sb.Append($"    \"{kvp.Key}\": {kvp.Value:G10}");
        }
        sb.AppendLine();
        sb.AppendLine("  }");
        sb.AppendLine("}");
        return sb.ToString();
    }
}

/// <summary>
/// Preset manager for storing multiple configurations.
/// </summary>
public class PresetManager
{
    private readonly Dictionary<string, ConfigurationPreset> _presets = new();
    private string _activePreset = "Default";

    public IReadOnlyDictionary<string, ConfigurationPreset> Presets => _presets;
    public string ActivePreset => _activePreset;

    public event Action<string>? PresetLoaded;
    public event Action<string>? PresetSaved;

    public void SavePreset(string name, ParameterManager manager, string description = "")
    {
        var preset = new ConfigurationPreset
        {
            Name = name,
            Description = description
        };
        preset.SaveFrom(manager);
        _presets[name] = preset;
        _activePreset = name;
        PresetSaved?.Invoke(name);
    }

    public bool LoadPreset(string name, ParameterManager manager)
    {
        if (_presets.TryGetValue(name, out var preset))
        {
            preset.ApplyTo(manager);
            _activePreset = name;
            PresetLoaded?.Invoke(name);
            return true;
        }
        return false;
    }

    public void DeletePreset(string name)
    {
        _presets.Remove(name);
    }

    public IEnumerable<string> GetPresetNames() => _presets.Keys;
}
