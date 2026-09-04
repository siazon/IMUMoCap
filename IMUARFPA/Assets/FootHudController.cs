using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class FootHudController : MonoBehaviour
{
    [Header("WebSocket")]
    public FootWebSocketClient wsClient;

    [Header("Left foot (IF footprint)")]
    public RectTransform leftSolid;
    public RectTransform leftOutline;
    public Image leftSolidImage;
    public Image leftOutlineImage;

    [Header("Right foot (IF footprint)")]
    public RectTransform rightSolid;
    public RectTransform rightOutline;
    public Image rightSolidImage;
    public Image rightOutlineImage;

    [Header("Footprint Root (IF)")]
    public GameObject footprintRoot;

    [Header("Stepping Stones (EF)")]
    public GameObject stoneRoot;
    public RectTransform stoneLeft;
    public RectTransform stoneRight;

    [Header("Overlay")]
    public GameObject overlayPanel;
    public TextMeshProUGUI overlayText;
    public Button confirmButton;
    public TextMeshProUGUI confirmButtonText;

    [Header("Progress Bar")]
    public Slider progressBar;

    private string _currentState = "disconnected";
    private string _condition = "";       // "" | "EF" | "IF"
    private int _currentBlock = 0;
    private float _targetL, _targetR;
    private string _directionL = "", _directionR = "";
    private bool _awaitingResumeFpa = false; // paused → resumed: show graphics again only on next fpa
    private float _restStartTime;
    private float _pendingRestDurationSec = 120f;

    // ── Unity lifecycle ───────────────────────────────────────────────────────

    void Start()
    {
        OnStateChange("disconnected");
    }

    // ── public API (called by FootWebSocketClient on main thread) ─────────────

    public void OnStateChange(string state) => OnStateChange(new ServerMessage { state = state });

    public void OnStateChange(ServerMessage msg)
    {
        string state = msg.state ?? "";
        Debug.Log($"[HUD] OnStateChange: {_currentState} → {state}");
        string previousState = _currentState;
        _currentState = state;

        if (!string.IsNullOrEmpty(msg.condition)) _condition = msg.condition;
        _currentBlock = msg.block;
        if (!string.IsNullOrEmpty(msg.directionL)) { _targetL = msg.targetL; _directionL = msg.directionL; }
        if (!string.IsNullOrEmpty(msg.directionR)) { _targetR = msg.targetR; _directionR = msg.directionR; }

        CancelInvoke(nameof(CheckRestElapsed));
        HideConfirmButton();
        SetProgressBar(false, 0f); // only baseline/retention show it, re-enabled in their case below

        if (previousState == "paused" && state != "paused")
            _awaitingResumeFpa = true;

        switch (state)
        {
            case "waiting":
                ShowTextGroup("Waiting... operator will start the condition.");
                break;

            case "armed":
                ShowTextGroup($"Ready ({_condition}).\nTap Start to begin calibration.");
                ShowConfirmButton("Start", OnStartCalibrationClicked);
                break;

            case "calibrating":
                ShowTextGroup("Calibrating... please stand still.");
                break;

            case "baseline":
                ShowTextGroup("Walk naturally to collect baseline data.");
                SetProgressBar(true, 0f);
                break;

            case "training":
                if (_awaitingResumeFpa)
                    ShowTextGroup("Resuming — keep walking...");
                else
                    ShowGraphicGroup();
                break;

            case "rest":
                _restStartTime = Time.time;
                _pendingRestDurationSec = msg.restDurationSec > 0 ? msg.restDurationSec : 120f;
                ShowTextGroup($"Rest — block {msg.block} complete.");
                InvokeRepeating(nameof(CheckRestElapsed), 0f, 1f);
                break;

            case "retention":
                _awaitingResumeFpa = false;
                ShowTextGroup("Retention — walk naturally.");
                SetProgressBar(true, 0f);
                break;

            case "paused":
                ShowTextGroup("Paused by operator.");
                break;

            case "ended":
                _awaitingResumeFpa = false;
                ShowTextGroup("Session complete. Thank you!");
                break;

            case "error":
                _awaitingResumeFpa = false;
                ShowTextGroup(string.IsNullOrEmpty(msg.reason) ? "Error — please restart the session." : $"Error — {msg.reason}");
                break;

            default: // "disconnected" or unknown
                ShowTextGroup("Connecting... please wait.");
                break;
        }
    }

    // Called for every "fpa" packet from the server. Sent only during a training block.
    public void OnFpaUpdate(ServerMessage msg)
    {
        if (msg.stage != "training")
        {
            Debug.LogWarning($"[HUD] OnFpaUpdate: ignored — stage is '{msg.stage}', expected 'training'");
            return;
        }

        if (_awaitingResumeFpa || _currentState != "training")
        {
            _awaitingResumeFpa = false;
            _currentState = "training";
            ShowGraphicGroup();
        }

        if (_condition == "EF")
            return; // stepping stone is a fixed target cue driven only by state.targetL/R — no per-step update.

        // IF: footprint rotates by errorL/errorR and colors by onTargetL/onTargetR, every step.
        if (!msg.fpaLIsNaN)
            UpdateFoot(leftSolid, leftOutline, leftSolidImage, leftOutlineImage, angle: msg.errorL, onTarget: msg.onTargetL);
        if (!msg.fpaRIsNaN)
            UpdateFoot(rightSolid, rightOutline, rightSolidImage, rightOutlineImage, angle: msg.errorR, onTarget: msg.onTargetR);
    }

    // Called for every "live" packet from the server — always shows the footprint, independent of EF/IF.
    // live has no on-target signal, so only rotation updates; color is left untouched.
    public void OnLiveUpdate(ServerMessage msg)
    {
        if (overlayPanel != null) overlayPanel.SetActive(false);
        if (stoneRoot != null) stoneRoot.SetActive(false);
        if (footprintRoot != null) footprintRoot.SetActive(true);

        if (!msg.fpaLIsNaN && leftSolid != null)
        {
            leftSolid.gameObject.SetActive(true);
            leftSolid.localRotation = Quaternion.Euler(0, 0, msg.angleL);
        }
        if (!msg.fpaRIsNaN && rightSolid != null)
        {
            rightSolid.gameObject.SetActive(true);
            rightSolid.localRotation = Quaternion.Euler(0, 0, msg.angleR);
        }
    }

    // Drives the step-count progress bar during baseline and retention.
    public void OnStepProgress(ServerMessage msg)
    {
        int totalSteps = msg.stepsL + msg.stepsR;
        int totalRequired = msg.requiredL + msg.requiredR;
        float progress = totalRequired > 0 ? Mathf.Clamp01((float)totalSteps / totalRequired) : 0f;
        SetProgressBar(true, progress);

        if (overlayText != null)
        {
            string label = msg.stage == "retention" ? "Retention" : "Baseline";
            overlayText.text = $"{label}\nL: {msg.stepsL} / {msg.requiredL}   R: {msg.stepsR} / {msg.requiredR}";
        }
    }

    // Operator redid the current stage — reset locally-tracked progress display.
    public void OnRedo(ServerMessage msg)
    {
        Debug.Log($"[HUD] OnRedo: stage={msg.stage} attempt={msg.attempt} reason={msg.reason}");
        if (overlayText != null)
            overlayText.text = $"Redo — {msg.stage} (attempt {msg.attempt})" +
                                (string.IsNullOrEmpty(msg.reason) ? "" : $"\n{msg.reason}");
    }

    // ── button handlers ──────────────────────────────────────────────────────

    private void OnStartCalibrationClicked()
    {
        Debug.Log($"[HUD] Start button clicked (wsClient={(wsClient != null)})");
        if (wsClient != null) _ = wsClient.SendReadyForCalibrationAsync();
        HideConfirmButton();
    }

    private void OnContinueClicked()
    {
        Debug.Log($"[HUD] Continue button clicked (wsClient={(wsClient != null)}, block={_currentBlock})");
        if (wsClient != null) _ = wsClient.SendContinueTrainingAsync(_currentBlock);
        HideConfirmButton();
    }

    private void CheckRestElapsed()
    {
        float remaining = Mathf.Max(0f, _pendingRestDurationSec - (Time.time - _restStartTime));
        if (overlayText != null)
            overlayText.text = remaining > 0
                ? $"Rest — block {_currentBlock} complete.\nContinue available in {Mathf.CeilToInt(remaining)}s."
                : "Rest complete. Tap Continue to proceed.";

        if (remaining <= 0f)
        {
            CancelInvoke(nameof(CheckRestElapsed));
            ShowConfirmButton("Continue", OnContinueClicked);
        }
    }

    // ── canvas group switching (FootprintRoot / StoneRoot / CalibOverlay) ──────

    private void ShowTextGroup(string message)
    {
        if (footprintRoot != null) footprintRoot.SetActive(false);
        if (stoneRoot != null) stoneRoot.SetActive(false);
        if (overlayPanel != null) overlayPanel.SetActive(true);
        if (overlayText != null) overlayText.text = message;
    }

    private void ShowGraphicGroup()
    {
        if (overlayPanel != null) overlayPanel.SetActive(false);
        bool isEF = _condition == "EF";
        bool isIF = _condition == "IF";
        if (stoneRoot != null) stoneRoot.SetActive(isEF);
        if (footprintRoot != null) footprintRoot.SetActive(isIF);
        if (isEF) ApplyStoneTargets();
    }

    private void ApplyStoneTargets()
    {
        if (stoneLeft != null) stoneLeft.localRotation = Quaternion.Euler(0, 0, SignedTargetAngle(_targetL, _directionL));
        if (stoneRight != null) stoneRight.localRotation = Quaternion.Euler(0, 0, SignedTargetAngle(_targetR, _directionR));
    }

    private static float SignedTargetAngle(float target, string direction) =>
        direction == "toe-in" ? -target : target;

    // ── helpers ───────────────────────────────────────────────────────────────

    private void UpdateFoot(RectTransform solid, RectTransform outline,
                            Image solidImg, Image outlineImg,
                            float angle, bool onTarget)
    {
        if (solid == null || solidImg == null) return;

        solid.gameObject.SetActive(true);
        solid.localRotation = Quaternion.Euler(0, 0, angle);
        solidImg.color = onTarget ? Color.green : Color.red;

        if (outline != null) outline.gameObject.SetActive(false);
        if (outlineImg != null) outlineImg.color = solidImg.color;
    }

    private void ShowConfirmButton(string label, UnityEngine.Events.UnityAction onClick)
    {
        if (confirmButton == null) return;
        confirmButton.gameObject.SetActive(true);
        confirmButton.onClick.RemoveAllListeners();
        confirmButton.onClick.AddListener(onClick);
        if (confirmButtonText != null) confirmButtonText.text = label;
    }

    private void HideConfirmButton()
    {
        if (confirmButton == null) return;
        confirmButton.gameObject.SetActive(false);
        confirmButton.onClick.RemoveAllListeners();
    }

    private void SetProgressBar(bool visible, float normalizedValue)
    {
        if (progressBar == null) return;
        progressBar.gameObject.SetActive(visible);
        if (visible)
        {
            progressBar.minValue = 0f;
            progressBar.maxValue = 1f;
            progressBar.value = normalizedValue;
        }
    }
}
