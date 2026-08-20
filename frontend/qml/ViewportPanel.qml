import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Mu.Material
import CameraTracking

ThemedCard {
    id: root
    property bool hasFramed: false

    Timer {
        id: framingSettle
        interval: 1500
        onTriggered: root.hasFramed = true
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 8

        RowLayout {
            Layout.fillWidth: true

            Label {
                Layout.fillWidth: true
                text: qsTr("Viewport")
                color: Theme.textPrimary
                font.bold: true
                font.pixelSize: 18
            }

            Label {
                text: robotScene.status
                color: Theme.textSecondary
                font.pixelSize: 12
                elide: Text.ElideMiddle
                Layout.maximumWidth: 240
            }

            MuButton {
                text: qsTr("Fit")
                enabled: viewer.loadedCount > 0
                Material.background: Theme.seMediumPurple
                Material.foreground: Theme.textPrimary
                onClicked: viewer.fitCamera()
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: Theme.radiusMedium
            color: Theme.surfaceInset
            border.color: Theme.outline
            border.width: 1
            clip: true

            MuMultiModelView {
                id: viewer
                anchors.fill: parent
                model: robotScene.model
                active: viewer.loadedCount > 0

                clearColor: Theme.surfaceInset

                frameUpAxis: Qt.vector3d(0, 0, 1)

                autoFit: !root.hasFramed


                onLoadedCountChanged: if (loadedCount > 0 && !root.hasFramed) framingSettle.restart()

                framePitch: backend.viewPitchDeg
                frameYaw: backend.viewYawDeg
                framePadding: backend.viewPadding

                modelColor: Theme.seGrey3   
                                            
            }

            StatusChip {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.margins: 10
                visible: backend.stale
                text: qsTr("NO SIGNAL")
                tone: Theme.critical
            }

            Text {
                anchors.centerIn: parent
                visible: viewer.loadedCount === 0
                text: robotScene.linkCount > 0 ? qsTr("Loading robot links…")
                                               : robotScene.status
                color: Theme.textSecondary
                wrapMode: Text.WordWrap
                width: parent.width * 0.72
                horizontalAlignment: Text.AlignHCenter
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 20

            Label {
                text: qsTr("anchor  %1").arg(backend.anchorFrame || "—")
                color: Theme.textSecondary
                opacity: backend.stale ? 0.4 : 1.0
                font.pixelSize: 12
            }

            Label {
                text: backend.placementSummary
                color: backend.sceneComplete ? Theme.textSecondary : Theme.caution
                opacity: backend.stale ? 0.4 : 1.0
                font.pixelSize: 12
            }

            Item { Layout.fillWidth: true }
        }
    }
}
