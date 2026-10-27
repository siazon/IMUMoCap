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

        /// <param name="redoAllowed">Redo restarts the stage being collected, so it only makes sense while the
        /// baseline / a training block / retention is running — not during rest, calibration or waiting for Ready.</param>
        public PauseResumeDialog(TimeSpan pauseElapsed, bool redoAllowed = true)
        {
            InitializeComponent();
            txtDuration.Text = $"Pause duration: {pauseElapsed.TotalMinutes:F1} min.";
            if (!redoAllowed)
            {
                btnRedo.IsEnabled = false;
                btnRedo.ToolTip = "Redo restarts the stage that is being collected. There is nothing to redo right now " +
                                  "(resting, calibrating or waiting for Ready) — use Continue.";
            }
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
