using System.Windows;

namespace IMUMoCap
{
    public partial class ParamsDialog : Window
    {
        public event RoutedEventHandler? SaveParamsClicked;

        public ParamsDialog()
        {
            InitializeComponent();
        }

        private void BtnSaveParams_Click(object sender, RoutedEventArgs e)
        {
            SaveParamsClicked?.Invoke(sender, e);
        }
    }
}
