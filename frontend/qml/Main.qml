import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Mu.Material
import CameraTracking

// Three screens, forward only: Home -> Rotor Placement -> Position Tracking.

MuApp {
    id: window
    width: 1280
    height: 900

    readonly property string baseTitle: qsTr("P2D2 — Optitrack Validation")
    title: backend.screenTitle ? baseTitle + ": " + backend.screenTitle : baseTitle

    color: Theme.background
    Material.theme: Material.Dark
    Material.background: Theme.background
    Material.foreground: Theme.textPrimary
    Material.primary: Theme.seMediumPurple
    Material.accent: Theme.seBrightPurple

    header: MuAppBar {
        title: window.title
        background: Rectangle { color: Theme.seMediumPurple }
    }

    ColumnLayout {
        id: shell
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: bannerLabel.implicitHeight + 12
            visible: !backend.jointsLive
            color: Theme.critical
            radius: Theme.radiusSmall

            Label {
                id: bannerLabel
                anchors.centerIn: parent
                text: qsTr("NO JOINT STATE: the arm is at its home pose, not a measured position")
                color: Theme.textOn(Theme.critical)
                font.bold: true
            }
        }
        ViewportPanel {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        StackLayout {
            id: bar
            currentIndex: backend.screen
            readonly property real share: Math.max(150, Math.min(320, shell.height * 0.30))

            Layout.fillWidth: true
            Layout.preferredHeight: share
            Layout.minimumHeight: share
            Layout.maximumHeight: share

            HomeBar {}
            RotorPlacementBar {}
            TrackingBar {}
        }
    }
}
