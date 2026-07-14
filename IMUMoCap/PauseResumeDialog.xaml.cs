using System;
using System.Windows;

namespace IMUMoCap
{
    /// <summary>
    /// 恢复暂停时弹出：不管选择继续还是重测，都必须填写暂停原因。
    /// </summary>
    public partial class PauseResumeDialog : Window
    {
        public string Reason { get; private set; } = "";
        public bool Redo { get; private set; }

        public PauseResumeDialog(TimeSpan pauseElapsed)
        {
            InitializeComponent();
            txtDuration.Text = $"Pause duration: {pauseElapsed.TotalMinutes:F1} min.";
        }

        private bool ValidateReason()
        {
            if (string.IsNullOrWhiteSpace(txtReason.Text))
            {
                txtError.Visibility = Visibility.Visible;
                return false;
            }
            txtError.Visibility = Visibility.Collapsed;
            return true;
        }

        private void BtnContinue_Click(object sender, RoutedEventArgs e)
        {
            if (!ValidateReason()) return;
            Reason = txtReason.Text.Trim();
            Redo = false;
            DialogResult = true;
        }

        private void BtnRedo_Click(object sender, RoutedEventArgs e)
        {
            if (!ValidateReason()) return;
            Reason = txtReason.Text.Trim();
            Redo = true;
            DialogResult = true;
        }
    }
}
