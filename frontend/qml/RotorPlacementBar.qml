import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import CameraTracking

// Screen 2: the startup review gate.

ThemedCard {
    id: root
    clip: true

    readonly property color tone: Theme.severityTone(backend.reviewSeverity)

    RowLayout {
        id: content
        anchors.fill: parent
        anchors.margins: 12
        spacing: 12

        // --- the message ----------------------------------------------------
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: Theme.radiusSmall
            color: Theme.surfaceInset
            border.width: 1
            border.color: root.tone

            ScrollView {
                id: messageScroll
                anchors.fill: parent
                anchors.margins: 10
                clip: true
                contentWidth: availableWidth

                ColumnLayout {
                    id: message
                    width: messageScroll.availableWidth
                    spacing: 6
                    Label {
                        Layout.fillWidth: true
                        text: backend.reviewShouldRecalibrate ? qsTr("DO NOT PROCEED — recalibration recommended") : qsTr("Placement within tolerance — safe to proceed")
                        color: backend.reviewShouldRecalibrate ? root.tone : Theme.ok
                        font.bold: true
                        font.pixelSize: 15
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 22

                        ColumnLayout {
                            spacing: 0
                            Label {
                                text: qsTr("%1 mm").arg(backend.reviewWorstMm.toFixed(2))
                                color: backend.reviewWorstMm >= backend.reviewWarnMm ? root.tone : Theme.textPrimary
                                font.bold: true
                                font.pixelSize: 24
                            }
                            Label {
                                text: qsTr("distance · limit %1").arg(backend.reviewWarnMm.toFixed(1))
                                color: Theme.textMuted
                                font.pixelSize: 10
                            }
                        }
                        ColumnLayout {
                            spacing: 0
                            Label {
                                text: qsTr("%1°").arg(backend.reviewWorstDeg.toFixed(3))
                                color: backend.reviewWorstDeg >= backend.reviewWarnDeg ? root.tone : Theme.textPrimary
                                font.bold: true
                                font.pixelSize: 24
                            }
                            Label {
                                text: qsTr("orientation · limit %1").arg(backend.reviewWarnDeg.toFixed(2))
                                color: Theme.textMuted
                                font.pixelSize: 10
                            }
                        }
                        Item {
                            Layout.fillWidth: true
                        }
                    }

                    Label {
                        Layout.fillWidth: true
                        text: backend.reviewShouldRecalibrate ? qsTr("The %1 disagreement between the measured and expected placement " + "is past the configured limit. Proceeding validates the hand " + "against a placement that is already suspect.").arg(backend.reviewVerdict) : qsTr("Measured and expected placements agree inside the configured " + "limits. These values are fixed for the rest of the session.")
                        color: Theme.textSecondary
                        wrapMode: Text.WordWrap
                        font.pixelSize: 11
                    }

                    Repeater {
                        model: backend.reviewComparisons
                        delegate: Label {
                            required property var modelData
                            Layout.fillWidth: true
                            text: qsTr("%1:  %2 mm  ∠ %3°   Δx %4  Δy %5  Δz %6 mm").arg(modelData.objectId).arg(modelData.distanceMm.toFixed(2)).arg(modelData.angleDeg.toFixed(3)).arg(modelData.dxMm.toFixed(1)).arg(modelData.dyMm.toFixed(1)).arg(modelData.dzMm.toFixed(1))
                            color: Theme.textMuted
                            font.pixelSize: 10
                            elide: Text.ElideRight
                        }
                    }

                    Label {
                        Layout.fillWidth: true
                        visible: backend.reviewComparisons.length > 0
                        text: qsTr("Read distance as the result — for a two-mount object the " + "fused orientation lies between its mounts.")
                        color: Theme.textMuted
                        wrapMode: Text.WordWrap
                        font.pixelSize: 10
                    }
                }
            }
        }

        ColumnLayout {
            Layout.preferredWidth: 200
            Layout.fillHeight: true
            spacing: 6

            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 38
                text: qsTr("Continue to pos validation")
                highlighted: !backend.reviewShouldRecalibrate
                onClicked: backend.acceptReview()
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 6

                Switch {
                    id: logSwitch
                    checked: backend.logRequested
                    onToggled: backend.logRequested = checked
                }
                Label {
                    Layout.fillWidth: true
                    text: logSwitch.checked ? qsTr("log position data") : qsTr("not logging")
                    color: logSwitch.checked ? Theme.textSecondary : Theme.caution
                    wrapMode: Text.WordWrap
                    font.pixelSize: 10
                }
            }

            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 38
                text: qsTr("Recalibrate P2D2")
                highlighted: backend.reviewShouldRecalibrate
            
                Material.accent: root.tone
                onClicked: backend.exitForRecalibration()
            }

            Label {
                Layout.fillWidth: true
                text: qsTr("Recalibrating closes the application.")
                color: Theme.textMuted
                wrapMode: Text.WordWrap
                font.pixelSize: 9
            }

            Item {
                Layout.fillHeight: true
            }
        }
    }
}
