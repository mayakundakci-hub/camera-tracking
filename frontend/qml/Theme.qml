pragma Singleton

import QtQuick

QtObject {
    // --- primary -----------------------------------------------------------
    readonly property color seDarkPurple:   "#1b1534"   //  27  21  52
    readonly property color seMediumPurple: "#4d217a"   //  77  33 122
    readonly property color seBrightPurple: "#8a00e5"   // 138   0 229

    // --- secondary: grey (light to dark) -----------------------------------
    readonly property color seGrey1: "#d7e4ee"          // 215 228 238
    readonly property color seGrey2: "#b8cedb"          // 184 206 219
    readonly property color seGrey3: "#96b0c0"          // 150 176 192
    readonly property color seGrey4: "#7b919d"          // 123 145 157

    // --- secondary: green (bright to dark) ---------------------------------
    readonly property color seGreen1: "#00fd79"         //   0 253 121
    readonly property color seGreen2: "#14da79"         //  20 218 121
    readonly property color seGreen3: "#27b66d"         //  39 182 109
    readonly property color seGreen4: "#009b55"         //   0 155  85

    // --- secondary: brown (light to dark) ----------------------------------
    readonly property color seBrown1: "#cfc0be"         // 207 192 190
    readonly property color seBrown2: "#af9e9c"         // 175 158 156
    readonly property color seBrown3: "#8f7c7a"         // 143 124 122
    readonly property color seBrown4: "#7d6766"         // 125 103 102

    // --- surfaces ----------------------------------------------------------
    // The elevation ramp is mixed along the dark-to-medium purple axis rather
    // than invented from greys, so raised surfaces stay inside the primaries.
    readonly property color background:   seDarkPurple  // window
    readonly property color surface:      "#251742"     // cards: dark + 20% medium
    readonly property color surfaceInset: "#1f163a"     // wells and rows: dark + 8% medium
    readonly property color outline:      seMediumPurple

    // --- text --------------------------------------------------------------
    readonly property color textPrimary:   seGrey1
    readonly property color textSecondary: seGrey3
    readonly property color textMuted:     seGrey4

    // --- status ------------------------------------------------------------

    readonly property color ok:       seGreen1
    readonly property color caution:  seBrown1
    readonly property color critical: seBrightPurple
    readonly property color neutral:  seGrey3

    function textOn(fill) {
        return (0.299 * fill.r + 0.587 * fill.g + 0.114 * fill.b) > 0.55 ? seDarkPurple : seGrey1
    }

    // --- severity ramp (the startup review gate) ----------------------------
    readonly property color severityLow:  Qt.lighter(seMediumPurple, 2.4)
    readonly property color severityHigh: seBrightPurple

    function severityTone(t) {
        var s = Math.max(0.0, Math.min(1.0, t))
        var a = severityLow
        var b = severityHigh
        return Qt.rgba(a.r + (b.r - a.r) * s,
                       a.g + (b.g - a.g) * s,
                       a.b + (b.b - a.b) * s, 1.0)
    }

    // --- type --------------------------------------------------------------
    readonly property string numericFamily: "Consolas"

    // --- geometry ----------------------------------------------------------
    readonly property int radiusSmall: 8
    readonly property int radiusMedium: 12
    readonly property int radiusLarge: 24
}
