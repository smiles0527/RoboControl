using System.Windows;
using System.Windows.Controls;
using ControlWorkbench.VEX.CodeGen;

namespace ControlWorkbench.App.Views;

/// <summary>
/// VEX Robot Configuration view - visual configuration wizard.
/// </summary>
public partial class VexRobotConfigView : UserControl
{
    private readonly RobotCodeGenerator _generator = new();

    public VexRobotConfigView()
    {
        InitializeComponent();
        GenerateButton.Click += GenerateButton_Click;
        PreviewFileSelect.SelectionChanged += PreviewFileSelect_SelectionChanged;
        
        // Initialize with default config
        UpdateConfig();
    }

    private void UpdateConfig()
    {
        _generator.Config = FullRobotConfig.Create6MotorBase();
        _generator.Config.TeamNumber = TeamNumber.Text;
        _generator.Config.RobotName = RobotName.Text;
        _generator.Config.UseLemLib = LibraryChoice.SelectedIndex == 0;
        _generator.Config.UseEzTemplate = LibraryChoice.SelectedIndex == 1;

        // Set gearset based on selection
        _generator.Config.DriveGearset = CartridgeType.SelectedIndex switch
        {
            0 => "red",
            1 => "green",
            2 => "blue",
            _ => "blue"
        };

        // Add intake
        _generator.Config.AddIntake(7);

        // Add pneumatics
        _generator.Config.AddPneumatic('A', "mogo_clamp", "pros::E_CONTROLLER_DIGITAL_L1");
        _generator.Config.AddPneumatic('B', "doinker", "pros::E_CONTROLLER_DIGITAL_L2");
    }

    private void GenerateButton_Click(object sender, RoutedEventArgs e)
    {
        UpdateConfig();

        try
        {
            var files = _generator.GenerateProsProject();
            
            MessageBox.Show(
                $"Generated {files.Count} files successfully!\n\n" +
                string.Join("\n", files.Keys.Take(8)) +
                (files.Count > 8 ? $"\n... and {files.Count - 8} more" : ""),
                "Code Generated",
                MessageBoxButton.OK,
                MessageBoxImage.Information);

            // Update preview
            UpdateCodePreview();
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Error generating code: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    private void PreviewFileSelect_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        UpdateCodePreview();
    }

    private void UpdateCodePreview()
    {
        UpdateConfig();
        var files = _generator.GenerateProsProject();

        string selectedFile = PreviewFileSelect.SelectedIndex switch
        {
            0 => "src/main.cpp",
            1 => "include/robot.hpp",
            2 => "src/robot.cpp",
            3 => "src/autons.cpp",
            _ => "src/main.cpp"
        };

        if (files.TryGetValue(selectedFile, out string? content))
        {
            CodePreviewBox.Text = content;
        }
    }
}
