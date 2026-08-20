import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import CameraTracking

// Screen 1's status bar: the scene coming up, and the one control that leaves.
ThemedCard {
    id: root
    clip: true

    RowLayout {
        id: content
        anchors.fill: parent
        anchors.margins: 12
        spacing: 16

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4

            RowLayout {
                Layout.fillWidth: true
                spacing: 8

                StatusChip {
                    text: backend.stale ? qsTr("BACKEND SILENT")
                                        : (backend.sceneComplete ? qsTr("scene complete")
                                                                 : qsTr("scene incomplete"))
                    tone: backend.stale ? Theme.critical
                                        : (backend.sceneComplete ? Theme.ok : Theme.caution)
                }
                StatusChip {
                    text: backend.placementSummary
                    tone: Theme.neutral
                }
                Label {
                    text: backend.anchorFrame
                          ? qsTr("%1 · %2").arg(backend.anchorFrame).arg(backend.anchorStatus)
                          : ""
                    color: backend.anchorStatus === "anchored" ? Theme.textSecondary
                                                               : Theme.caution
                    font.pixelSize: 11
                }
                Item { Layout.fillWidth: true }
            }

            Label {
                Layout.fillWidth: true
                text: backend.readyToBegin
                      ? qsTr("Placements resolved. Begin to review them before tracking starts.")
                      : qsTr("Waiting — %1. The console's [latch] report says how each "
                             + "placement is progressing.").arg(backend.beginBlockedReason)
                color: backend.readyToBegin ? Theme.textSecondary : Theme.caution
                wrapMode: Text.WordWrap
                font.pixelSize: 11
            }
        }

        Button {
            text: qsTr("Begin")
            enabled: backend.readyToBegin
            highlighted: backend.readyToBegin
            Layout.preferredWidth: 150
            Layout.preferredHeight: 44
            onClicked: backend.begin()
        }
    }
}
