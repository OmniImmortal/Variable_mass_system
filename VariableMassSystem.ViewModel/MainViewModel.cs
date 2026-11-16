using CommunityToolkit.Mvvm.ComponentModel;

namespace VariableMassSystem.ViewModel;

public partial class MainViewModel : ViewModelBase
{
    [ObservableProperty]
    public string _greeting = "Welcome to Avalonia!";
}
