using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class FootHudController : MonoBehaviour
{
    [Header("WebSocket")]
    public FootWebSocketClient wsClient;

    [Header("Left foot (IF footprint)")]
    public RectTransform leftSolid;
    public Image leftSolidImage;

    [Header("Right foot (IF footprint)")]
    public RectTransform rightSolid;
    public Image rightSolidImage;

    [Header("Footprint Root (IF)")]
    public GameObject footprintRoot;

    [Header("Stepping Stones (EF)")]
    public GameObject stoneRoot;
    public RectTransform stoneLeft;
    public Image stoneLeftImage;
    public RectTransform stoneRight;
    public Image stoneRightImage;

    [Header("Overlay")]
    public GameObject overlayPanel;
    public TextMeshProUGUI overlayText;
    public Button confirmButton;
    public TextMeshProUGUI confirmButtonText;

    [Header("Focus Reminder")]
    // Overlaid below the footprint/stone graphic (which is already visible) for
    // focusReminderDurationSec at the start of every training block, then hidden for the rest of
    // the block — matches EF/IF on both duration and word count so neither condition carries a
    // persistent on-screen instruction the other doesn't.
    public GameObject focusReminderPanel;
    public TextMeshProUGUI focusReminderText;
    public float focusReminderDurationSec = 3f;

    [Header("IF Live Mode")]
    // If no "live"/"fpa" update is received for a foot within this many seconds while in IF live
    // mode, that foot's icon reverts to the idle pose so it never gets stuck showing a stale angle.
    public float liveStaleThresholdSec = 0.5f;

    [Header("Progress Bar")]
    public Slider progressBar;

    // Terminal-flash display duration — shared by EF (stone pulse) and IF (footprint icon) so both
    // conditions show their per-step feedback for the exact same wall-clock time.
    // Recorded-walk analysis (docs/task8_footprint_display_duration.py on
    // ImuSamples_20260817_162335.csv) found same-foot step-to-step gap ~0.46-0.5s (p10-median),
    // so 1s never finished hiding before the next step re-triggered it; 0.35s still finishes
    // comfortably before the next same-foot step and tested well manually.
    private const float FootOutDisplaySec = 0.35f;

    // "Slightly enlarged" pulse scale applied to the stone/foot icon on each step (both hit and
    // miss) — shared by EF's PulseStone and IF's FlashFoot.
    private const float PulseScale = 1.05f;

    // Word-count-matched (10 words each) so neither condition's pre-block instruction is
    // measurably longer/more complex to read than the other's.
    private const string FocusReminderTextEF = "Focus on placing your foot onto the stepping stones ahead.";
    private const string FocusReminderTextIF = "Focus on watching your own foot direction on the display.";

    private string _currentState = "disconnected";
    private string _condition = "";       // "" | "EF" | "IF"
    private int _currentBlock = 0;
    private float _targetL, _targetR;
    private string _directionL = "", _directionR = "";
    private bool _awaitingResumeFpa = false; // paused → resumed: show graphics again only on next fpa
    private float _restStartTime;
    private float _pendingRestDurationSec = 120f;
    private Coroutine _focusReminderCoroutine;
    private Color _stoneLeftNormalColor = Color.white;
    private Color _stoneRightNormalColor = Color.white;
    private Color _leftSolidNormalColor = Color.white;
    private Color _rightSolidNormalColor = Color.white;

    // IF render mode ("placeholder" | "live" | "lastResult"), driven by state.ifRenderMode — see
    // OnStateChange. EF is entirely unaffected by this; only the OnFpaUpdate/OnLiveUpdate paths
    // below check it. Default "lastResult" — matches the PC side's default-checked ChkIfLastResultMode.
    private string _ifRenderMode = "lastResult";

    // Time.time of the most recent live/fpa update per foot, while in IF live mode — used by
    // Update() to revert a foot to its idle pose if it goes stale (see liveStaleThresholdSec).
    // -1 means "no activity yet this block", so the staleness check in Update() is skipped.
    private float _lastLeftLiveActivityTime = -1f;
    private float _lastRightLiveActivityTime = -1f;

    // ── Unity lifecycle ───────────────────────────────────────────────────────

    void Start()
    {
        if (stoneLeftImage != null) _stoneLeftNormalColor = stoneLeftImage.color;
        if (stoneRightImage != null) _stoneRightNormalColor = stoneRightImage.color;
        if (leftSolidImage != null) _leftSolidNormalColor = leftSolidImage.color;
        if (rightSolidImage != null) _rightSolidNormalColor = rightSolidImage.color;
        OnStateChange("disconnected");
    }

    // IF live mode only: revert a foot to its idle pose if it hasn't heard a live/fpa update in
    // a while (e.g. that foot is airborne/inactive) — avoids it getting stuck on a stale angle.
    // No-op in placeholder mode and for EF (ApplyStoneTargets/PulseStone never set these timers).
    void Update()
    {
        if (_condition != "IF" || _ifRenderMode != "live") return;

        if (_lastLeftLiveActivityTime >= 0f && Time.time - _lastLeftLiveActivityTime > liveStaleThresholdSec)
        {
            ResetLeftSolid();
            _lastLeftLiveActivityTime = -1f;
        }
        if (_lastRightLiveActivityTime >= 0f && Time.time - _lastRightLiveActivityTime > liveStaleThresholdSec)
        {
            ResetRightSolid();
            _lastRightLiveActivityTime = -1f;
        }
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
        // Fallback to "lastResult" when absent (old/un-upgraded PC server, or JsonUtility's
        // default null for an unset string field) — matches the current shipped default.
        _ifRenderMode = string.IsNullOrEmpty(msg.ifRenderMode) ? "lastResult" : msg.ifRenderMode;
        _currentBlock = msg.block;
        if (!string.IsNullOrEmpty(msg.directionL)) { _targetL = msg.targetL; _directionL = msg.directionL; }
        if (!string.IsNullOrEmpty(msg.directionR)) { _targetR = msg.targetR; _directionR = msg.directionR; }

        CancelInvoke(nameof(CheckRestElapsed));
        HideConfirmButton();
        SetProgressBar(false, 0f); // only baseline/retention show it, re-enabled in their case below

        // Any state change interrupts a pending focus-reminder countdown (e.g. operator hit Redo
        // mid-reminder) — never leave a stale reminder coroutine running into the wrong state.
        if (_focusReminderCoroutine != null)
        {
            StopCoroutine(_focusReminderCoroutine);
            _focusReminderCoroutine = null;
        }
        if (focusReminderPanel != null) focusReminderPanel.SetActive(false);

        if (previousState == "paused" && state != "paused")
            _awaitingResumeFpa = true;

        switch (state)
        {
            case "waiting":
                ShowTextGroup("Waiting... operator will start the condition.");
                break;

            case "armed":
                // reason carries a recalibration-specific prompt ("Quick recheck...") when this is a
                // pre-stage recalibration rather than the session's first calibration — see MainWindow's
                // AdvanceStage()/_pendingStageEntry.
                ShowTextGroup(string.IsNullOrEmpty(msg.reason) ? $"Ready ({_condition}).\nTap Ready to begin calibration." : msg.reason);
                ShowConfirmButton("Ready", OnStartCalibrationClicked);
                break;

            case "calibrating":
                ShowTextGroup(string.IsNullOrEmpty(msg.reason) ? "Calibrating... please stand still." : msg.reason);
                break;

            case "baseline":
                ShowTextGroup("Walk naturally to collect baseline data.");
                SetProgressBar(true, 0f);
                break;

            case "training":
                if (_awaitingResumeFpa)
                    ShowTextGroup("Resuming — keep walking...");
                else
                {
                    ShowGraphicGroup();
                    _focusReminderCoroutine = StartCoroutine(ShowFocusReminderThenHide());
                }
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

    // Called for every "fpa" packet from the server. Sent only during a training block, once per
    // completed stance (foot-contact) — this is the ONLY signal driving foot-icon updates: both
    // EF and IF are visible for the entire block (dwell-time matched) and only pulse/rotate+color
    // on contact, reverting to their idle pose after FootOutDisplaySec. There is no per-frame
    // "continuously active" feedback in either condition.
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
        {
            // stone target angle is fixed (set once from state.targetL/R); per-step feedback is a
            // brief scale-up pulse on the stone itself, tinted blue/orange on hit/miss — matches
            // IF's FeedbackColors palette.
            if (!msg.fpaLIsNaN) PulseStone(stoneLeft, stoneLeftImage, msg.onTargetL, nameof(ResetStoneLeft));
            if (!msg.fpaRIsNaN) PulseStone(stoneRight, stoneRightImage, msg.onTargetR, nameof(ResetStoneRight));
            return;
        }

        // IF: foot icon is visible for the whole block (see ShowGraphicGroup) at an idle pose;
        // on each fpa message it rotates+colors to errorL/errorR via FlashFoot (same window and
        // mechanism as EF's stone pulse above) — what happens after FootOutDisplaySec depends on
        // _ifRenderMode: see PulseIfSolid.
        if (!msg.fpaLIsNaN) PulseIfSolid(leftSolid, leftSolidImage, msg.errorL, msg.onTargetL, nameof(ResetLeftSolid), nameof(RevertLeftSolidColor), nameof(ResetLeftSolidScale), ref _lastLeftLiveActivityTime);
        if (!msg.fpaRIsNaN) PulseIfSolid(rightSolid, rightSolidImage, msg.errorR, msg.onTargetR, nameof(ResetRightSolid), nameof(RevertRightSolidColor), nameof(ResetRightSolidScale), ref _lastRightLiveActivityTime);
    }

    // No-op Invoke target: used as PulseIfSolid's revert method in "lastResult" mode, where a
    // step's rotate+color pulse is meant to persist indefinitely (until the next fpa overwrites
    // it) rather than revert — see PulseIfSolid.
    private void NoRevert() { }

    // Called for every "live" packet — IF-only real-time swing-phase angle reference (10Hz,
    // see ServerMessage.angleL/R for reference-frame caveats), only acted on in IF + live mode
    // (placeholder mode and EF ignore it entirely — this never touches stoneLeft/stoneRight).
    // Rotates straight to angleL/angleR without waiting for fpa and without touching color — live
    // carries no on-target signal, so color is left at whatever it already is (idle, or still
    // mid a recent fpa pulse).
    public void OnLiveUpdate(ServerMessage msg)
    {
        if (_condition != "IF" || _ifRenderMode != "live") return;

        if (!msg.angleLIsNaN && leftSolid != null)
        {
            leftSolid.localRotation = Quaternion.Euler(0, 0, msg.angleL);
            _lastLeftLiveActivityTime = Time.time;
        }
        if (!msg.angleRIsNaN && rightSolid != null)
        {
            rightSolid.localRotation = Quaternion.Euler(0, 0, msg.angleR);
            _lastRightLiveActivityTime = Time.time;
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

    // Rest no longer ends on a participant "Continue" tap — the PC auto-advances once the minimum
    // duration elapses (its own stopwatch, not this countdown) straight into the pre-stage
    // recalibration prompt ("armed"/"Tap Ready"), which already serves as the resume signal. This
    // just reflects the countdown locally; no action needed once it reaches zero.
    private void CheckRestElapsed()
    {
        float remaining = Mathf.Max(0f, _pendingRestDurationSec - (Time.time - _restStartTime));
        if (overlayText != null)
            overlayText.text = remaining > 0
                ? $"Rest — block {_currentBlock} complete.\nResuming in {Mathf.CeilToInt(remaining)}s."
                : "Rest complete. Preparing recalibration...";

        if (remaining <= 0f)
        {
            CancelInvoke(nameof(CheckRestElapsed));
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

    // Overlays the EF/IF-appropriate focus-reminder instruction below the (already-visible)
    // footprint/stone graphic for focusReminderDurationSec, then hides just the reminder text —
    // runs once at the start of every training block (not on resume-from-pause, which keeps its
    // own "Resuming..." text instead).
    private System.Collections.IEnumerator ShowFocusReminderThenHide()
    {
        if (focusReminderPanel != null)
        {
            if (focusReminderText != null)
                focusReminderText.text = _condition == "EF" ? FocusReminderTextEF : FocusReminderTextIF;
            focusReminderPanel.SetActive(true);
        }

        yield return new WaitForSeconds(focusReminderDurationSec);

        if (focusReminderPanel != null) focusReminderPanel.SetActive(false);
        _focusReminderCoroutine = null;
    }

    private void ShowGraphicGroup()
    {
        if (overlayPanel != null) overlayPanel.SetActive(false);
        bool isEF = _condition == "EF";
        bool isIF = _condition == "IF";
        if (stoneRoot != null) stoneRoot.SetActive(isEF);
        if (footprintRoot != null) footprintRoot.SetActive(isIF);

        if (isEF)
        {
            ApplyStoneTargets();
            // Stones start at normal scale/color — only OnFpaUpdate's per-step pulse changes them.
            ResetStoneLeft();
            ResetStoneRight();
        }
        if (isIF)
        {
            // Dwell-time matched to EF: visible for the whole block at an idle pose (zero
            // rotation, cached texture color) rather than hidden-by-default. This idle pose is a
            // placeholder to equalize on-screen dwell time with EF's always-visible stone — it
            // does NOT represent any real-time angle measurement. Only OnFpaUpdate's per-step
            // pulse (rotate+color to the actual errorL/errorR) changes it — in all three render
            // modes (placeholder/live/lastResult), since none of them has a previous result yet
            // at block start. In live mode this also establishes the required zero-rotation
            // starting pose before the first live/fpa update arrives; the activity timers are
            // cleared so Update()'s staleness check doesn't act on a stale timestamp left over
            // from a previous block.
            ResetLeftSolid();
            ResetRightSolid();
            _lastLeftLiveActivityTime = -1f;
            _lastRightLiveActivityTime = -1f;
        }
    }

    private void ApplyStoneTargets()
    {
        if (stoneLeft != null) stoneLeft.localRotation = Quaternion.Euler(0, 0, SignedTargetAngle(_targetL, _directionL));
        if (stoneRight != null) stoneRight.localRotation = Quaternion.Euler(0, 0, SignedTargetAngle(_targetR, _directionR));
    }

    private static float SignedTargetAngle(float target, string direction) =>
        direction == "toe-in" ? -target : target;

    private void ResetStoneLeft()
    {
        if (stoneLeft != null) stoneLeft.localScale = Vector3.one;
        if (stoneLeftImage != null) stoneLeftImage.color = _stoneLeftNormalColor;
    }

    private void ResetStoneRight()
    {
        if (stoneRight != null) stoneRight.localScale = Vector3.one;
        if (stoneRightImage != null) stoneRightImage.color = _stoneRightNormalColor;
    }

    // Reverts to the idle pose (zero rotation, cached texture color) — not a hide/SetActive, the
    // icon stays visible for the whole block. Used both to establish the idle pose at block start
    // (ShowGraphicGroup) and to revert after each fpa pulse (FlashFoot's resetMethodName).
    private void ResetLeftSolid()
    {
        if (leftSolid != null) { leftSolid.localRotation = Quaternion.identity; leftSolid.localScale = Vector3.one; }
        if (leftSolidImage != null) leftSolidImage.color = _leftSolidNormalColor;
    }

    private void ResetRightSolid()
    {
        if (rightSolid != null) { rightSolid.localRotation = Quaternion.identity; rightSolid.localScale = Vector3.one; }
        if (rightSolidImage != null) rightSolidImage.color = _rightSolidNormalColor;
    }

    // Scale-only revert — unlike ResetLeftSolid/RightSolid, leaves rotation/color alone. The
    // "slightly enlarged" pulse (see FlashFoot) is always transient regardless of _ifRenderMode,
    // same as EF's PulseStone; only rotation/color persistence varies by render mode.
    private void ResetLeftSolidScale() { if (leftSolid != null) leftSolid.localScale = Vector3.one; }
    private void ResetRightSolidScale() { if (rightSolid != null) rightSolid.localScale = Vector3.one; }

    // Live-mode-only pulse revert: unlike ResetLeftSolid/RightSolid, this leaves rotation alone
    // (it keeps tracking the live stream — see OnLiveUpdate) and only fades the fpa-pulse color
    // back to normal, per the "回落到继续跟随下一次 live 流的状态" requirement.
    private void RevertLeftSolidColor() { if (leftSolidImage != null) leftSolidImage.color = _leftSolidNormalColor; }
    private void RevertRightSolidColor() { if (rightSolidImage != null) rightSolidImage.color = _rightSolidNormalColor; }

    // ── helpers ───────────────────────────────────────────────────────────────

    // Pulses one foot's icon on contact: rotate to angle, scale up to PulseScale (like EF's
    // stone), color by onTarget — then after FootOutDisplaySec, the scale always reverts via
    // scaleResetMethodName (ResetLeftSolid/RightSolidScale, unconditional, same as EF) while
    // rotation/color revert via resetMethodName, whose target is mode-dependent (see
    // PulseIfSolid). Used by IF, sharing FootOutDisplaySec and the FeedbackColors palette with
    // EF's PulseStone below.
    private void FlashFoot(RectTransform solid, Image solidImg, float angle, bool onTarget, string resetMethodName, string scaleResetMethodName)
    {
        if (solid == null || solidImg == null)
        {
            Debug.LogWarning("[HUD] FlashFoot: solid/solidImg not assigned in Inspector — skipping.");
            return;
        }

        solid.gameObject.SetActive(true);
        solid.localRotation = Quaternion.Euler(0, 0, angle);
        solid.localScale = Vector3.one * PulseScale;
        solidImg.color = onTarget ? FeedbackColors.OnTarget : FeedbackColors.OffTarget;

        CancelInvoke(resetMethodName);
        Invoke(resetMethodName, FootOutDisplaySec);

        CancelInvoke(scaleResetMethodName);
        Invoke(scaleResetMethodName, FootOutDisplaySec);
    }

    // IF's fpa-triggered pulse: rotate+scale+color via FlashFoot — but which auto-revert
    // target it uses, and whether this also counts as "activity" for Update()'s live-mode
    // staleness check, depends on _ifRenderMode:
    //   - placeholder: fully reverts to the idle pose (ResetLeftSolid/RightSolid — rotation
    //     included) after FootOutDisplaySec.
    //   - live: only fades the color back, leaving rotation to keep tracking the live stream
    //     (OnLiveUpdate) until the next fpa/live update.
    //   - lastResult: never reverts (NoRevert) — the pulse's rotation+color is the sticky
    //     "last step's result" display, held as-is until the next fpa message overwrites it.
    private void PulseIfSolid(RectTransform solid, Image solidImg, float angle, bool onTarget,
        string placeholderResetMethodName, string liveColorRevertMethodName, string scaleResetMethodName, ref float liveActivityTime)
    {
        string resetMethodName = _ifRenderMode switch
        {
            "live" => liveColorRevertMethodName,
            "lastResult" => nameof(NoRevert),
            _ => placeholderResetMethodName,
        };
        FlashFoot(solid, solidImg, angle, onTarget, resetMethodName, scaleResetMethodName);
        if (_ifRenderMode == "live") liveActivityTime = Time.time;
    }

    // Shows one stone's terminal pulse: scale up, tint by onTarget (same FeedbackColors palette
    // as IF's flash), then auto-revert via resetMethodName (ResetStoneLeft/Right) after
    // FootOutDisplaySec.
    private void PulseStone(RectTransform stone, Image stoneImg, bool onTarget, string resetMethodName)
    {
        if (stone == null)
        {
            Debug.LogWarning("[HUD] PulseStone: stone RectTransform not assigned in Inspector — skipping.");
            return;
        }

        stone.localScale = Vector3.one * PulseScale;
        if (stoneImg != null) stoneImg.color = onTarget ? FeedbackColors.OnTarget : FeedbackColors.OffTarget;

        CancelInvoke(resetMethodName);
        Invoke(resetMethodName, FootOutDisplaySec);
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
