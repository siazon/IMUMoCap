using UnityEngine;

// Shared on-target/off-target feedback palette, used identically by EF (stone pulse) and
// IF (leftSolid/rightSolid flash) so a future palette change only happens in one place.
// Colorblind-safe blue/orange pair (Wong, 2011, Nature Methods, DOI: 10.1038/nmeth.1618) —
// replaces the earlier red/green pair, which is difficult to distinguish under red-green
// color-vision deficiency.
//
// MAPPING SWAPPED 2026-10-23 (pre-Stage-1-lock pilot finding): an n=3 perceptual pilot
// (colour_salience_pilot_trials.csv / colour_salience_pilot_subjective.csv, in
// 03_Preregistration/) found that (a) orange is consistently more visually salient than
// blue in situ (confirmed independently by in-situ CIEDE2000 photo analysis, see
// Preregistration_Draft.md Section 6), and (b) all 3 pilot participants, asked without
// being told the system's mapping, intuitively associated orange with a positive/success
// outcome and blue with a negative/failure outcome -- the OPPOSITE of the original
// OnTarget=blue/OffTarget=orange mapping. The mapping below has been swapped
// (OnTarget=orange, OffTarget=blue) so the more salient colour matches the valence
// participants naturally associate with success, rather than fighting their intuition.
// This does not affect colorblind-safety: the Wong (2011) blue/orange pair remains
// equally distinguishable under red-green colour-vision deficiency regardless of which
// one is assigned to which outcome.
//
// BACK TO GREEN/RED 2026-11-20 (pilot feedback): blue/orange carries no built-in meaning and
// participants kept mixing up which one meant on-target; green = on target, red = off target
// needs no learning. Colour-vision deficiency is handled by the visual screening instead:
// its colour pages ask the participant to name each colour, so someone who can't tell red
// from green fails the screening before the first condition.
public static class FeedbackColors
{
    public static readonly Color OnTarget = Color.green;
    public static readonly Color OffTarget = Color.red;
}
