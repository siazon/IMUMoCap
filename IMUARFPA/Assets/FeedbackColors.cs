using UnityEngine;

// Shared on-target/off-target feedback palette, used identically by EF (stone pulse) and
// IF (leftSolid/rightSolid flash) so a future palette change only happens in one place.
// Colorblind-safe blue/orange pair (Wong, 2011, Nature Methods, DOI: 10.1038/nmeth.1618) —
// replaces the earlier red/green pair, which is difficult to distinguish under red-green
// color-vision deficiency.
public static class FeedbackColors
{
    public static readonly Color OnTarget = new Color32(0, 114, 178, 255);
    public static readonly Color OffTarget = new Color32(230, 159, 0, 255);
}
