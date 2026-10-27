using System.ComponentModel;
using System.Windows.Media;

namespace IMUMoCap.Model
{
    public enum ChecklistStatus { Pending, Done, Missing }

    /// <summary>
    /// One row of the experiment-flow-at-a-glance checklist (MainWindow "Flow" panel).
    /// Status and CanMark are recomputed wholesale from ProtocolFlow by MainWindow.RefreshChecklist()
    /// rather than pushed from scattered call sites, so it can never drift out of sync.
    /// </summary>
    public sealed class ChecklistItem : INotifyPropertyChanged
    {
        public string Label { get; }

        // Non-null only for the 3 rows backed by an operator-clicked audit marker (NASA-TLX/IMI-PC).
        // The stage string MarkStageEvent(...) expects, e.g. "NASA-TLX_Admin1" — distinct from the
        // shorter display Label ("NASA-TLX #1") so the click handler doesn't have to reverse-parse it.
        public string? MarkerStage { get; }
        public bool IsMarkable => MarkerStage != null;

        // Vertical flowchart: whether a connector line is drawn from this dot down to the next one (false for
        // the last row). Set once after the list is built (MainWindow constructor); never changes afterwards.
        public bool HasNext { get; set; }

        private ChecklistStatus _status = ChecklistStatus.Pending;
        public ChecklistStatus Status
        {
            get => _status;
            set
            {
                if (_status == value) return;
                _status = value;
                OnPropertyChanged(nameof(Status));
                OnPropertyChanged(nameof(DotBrush));
                OnPropertyChanged(nameof(LineBrush));
            }
        }

        // Whether the inline Mark button (only rendered when IsMarkable) is currently clickable — mirrors
        // the window the marker belongs to being open (ProtocolFlow.CanMark, applied by MainWindow.RefreshChecklist).
        private bool _canMark = false;
        public bool CanMark
        {
            get => _canMark;
            set
            {
                if (_canMark == value) return;
                _canMark = value;
                OnPropertyChanged(nameof(CanMark));
            }
        }

        // Pending = not reached yet, Done = complete, Missing = window reached but a manual
        // audit marker (NASA-TLX/IMI-PC) hasn't been clicked yet — needs attention, not fatal.
        public Brush DotBrush => Status switch
        {
            ChecklistStatus.Done => Brushes.LimeGreen,
            ChecklistStatus.Missing => Brushes.Orange,
            _ => Brushes.Gray,
        };

        // Connector from this row's dot down to the next row's: green once this step is done, gray otherwise
        // (a "Missing" step hasn't been completed yet, so its connector stays gray too).
        public Brush LineBrush => Status == ChecklistStatus.Done ? Brushes.LimeGreen : Brushes.LightGray;

        public ChecklistItem(string label, string? markerStage = null)
        {
            Label = label;
            MarkerStage = markerStage;
        }

        public event PropertyChangedEventHandler? PropertyChanged;
        private void OnPropertyChanged(string propertyName) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
