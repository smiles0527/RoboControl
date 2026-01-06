using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;

namespace ControlWorkbench.App;

/// <summary>
/// Main window with welcome screen and VEX/Drone mode selection.
/// </summary>
public partial class MainWindow : Window
{
    private enum AppMode { None, Vex, Drone }
    private AppMode _currentMode = AppMode.None;
    
    public MainWindow()
    {
        InitializeComponent();
        
        // Ensure window gets focus when loaded
        Loaded += MainWindow_Loaded;
        
        // Start with welcome screen visible
        ShowWelcomeScreen();
    }
    
    private void MainWindow_Loaded(object sender, RoutedEventArgs e)
    {
        // Activate and focus the window
        Activate();
        Focus();
        
        // Set keyboard focus to the window
        Keyboard.Focus(this);
    }
    
    // ========== Welcome Screen ==========
    
    private void ShowWelcomeScreen()
    {
        WelcomeScreen.Visibility = Visibility.Visible;
        MainContent.Visibility = Visibility.Collapsed;
        _currentMode = AppMode.None;
    }
    
    private void VexOption_Click(object sender, MouseButtonEventArgs e)
    {
        EnterVexMode();
    }
    
    private void DroneOption_Click(object sender, MouseButtonEventArgs e)
    {
        EnterDroneMode();
    }
    
    private void Option_MouseEnter(object sender, MouseEventArgs e)
    {
        if (sender is Border border)
        {
            // Scale up and enhance glow
            var transform = new ScaleTransform(1.02, 1.02);
            border.RenderTransform = transform;
            border.RenderTransformOrigin = new Point(0.5, 0.5);
            
            // Increase glow intensity
            if (border.Effect is DropShadowEffect shadow)
            {
                shadow.Opacity = 0.4;
                shadow.BlurRadius = 30;
            }
        }
    }
    
    private void Option_MouseLeave(object sender, MouseEventArgs e)
    {
        if (sender is Border border)
        {
            // Reset scale
            border.RenderTransform = new ScaleTransform(1.0, 1.0);
            
            // Reset glow
            if (border.Effect is DropShadowEffect shadow)
            {
                shadow.Opacity = 0.15;
                shadow.BlurRadius = 20;
            }
        }
    }
    
    // ========== Mode Selection ==========
    
    private void EnterVexMode()
    {
        _currentMode = AppMode.Vex;
        
        // Hide welcome, show main content
        WelcomeScreen.Visibility = Visibility.Collapsed;
        MainContent.Visibility = Visibility.Visible;
        
        // Show VEX tabs, hide Drone tabs
        VexTabControl.Visibility = Visibility.Visible;
        DroneTabControl.Visibility = Visibility.Collapsed;
        
        // Update UI
        CurrentModeTitle.Text = "VEX V5 ROBOTICS";
        ModeStatusText.Text = "Platform: VEX V5";
        Title = "ControlWorkbench - VEX V5";
        
        // Update port options for VEX (serial ports)
        UpdatePortsForVex();
    }
    
    private void EnterDroneMode()
    {
        _currentMode = AppMode.Drone;
        
        // Hide welcome, show main content
        WelcomeScreen.Visibility = Visibility.Collapsed;
        MainContent.Visibility = Visibility.Visible;
        
        // Show Drone tabs, hide VEX tabs
        VexTabControl.Visibility = Visibility.Collapsed;
        DroneTabControl.Visibility = Visibility.Visible;
        
        // Update UI
        CurrentModeTitle.Text = "DRONE / UAV";
        ModeStatusText.Text = "Platform: Drone";
        Title = "ControlWorkbench - Drone";
        
        // Update port options for Drone (UDP, TCP, serial)
        UpdatePortsForDrone();
    }
    
    private void BackButton_Click(object sender, RoutedEventArgs e)
    {
        // Go back to welcome screen
        ShowWelcomeScreen();
        Title = "ControlWorkbench";
    }
    
    // ========== Port Configuration ==========
    
    private void UpdatePortsForVex()
    {
        PortSelector.Items.Clear();
        PortSelector.Items.Add("COM3");
        PortSelector.Items.Add("COM4");
        PortSelector.Items.Add("COM5");
        PortSelector.Items.Add("COM6");
        PortSelector.Items.Add("COM7");
        PortSelector.Items.Add("COM8");
        if (PortSelector.Items.Count > 0)
            PortSelector.SelectedIndex = 0;
    }
    
    private void UpdatePortsForDrone()
    {
        PortSelector.Items.Clear();
        PortSelector.Items.Add("UDP:14550");
        PortSelector.Items.Add("UDP:14540");
        PortSelector.Items.Add("TCP:5760");
        PortSelector.Items.Add("COM3");
        PortSelector.Items.Add("COM4");
        if (PortSelector.Items.Count > 0)
            PortSelector.SelectedIndex = 0;
    }
}