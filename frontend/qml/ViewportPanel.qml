import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Mu.Material

MuCard {
    id: root

    ColumnLayout {
        anchors.fill: parent
        spacing: 8

        RowLayout {
            Layout.fillWidth: true

            Label {
                Layout.fillWidth: true
                text: qsTr("Viewport")
                font.bold: true
                font.pixelSize: 18
            }

            Label {
                text: robotScene.status
                opacity: 0.6
                font.pixelSize: 12
                elide: Text.ElideMiddle
                Layout.maximumWidth: 240
            }

            MuButton {
                text: qsTr("Fit")
                role: "tonal"
                enabled: viewer.loadedCount > 0
                onClicked: viewer.fitCamera()
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 12
            color: "#151225"
            border.color: "#6b2696"
            border.width: 1
            clip: true

            MuMultiModelView {
                id: viewer
                anchors.fill: parent
                model: robotScene.model
                active: viewer.loadedCount > 0
                modelColor: "#c9a227"   // fallback only: links whose URDF visual has a
                                        // <material><color> render with that color via the
                                        // model's per-row "color" role; this yellow covers
                                        // links with no material
            }

            Label {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.margins: 10
                visible: backend.stale || !backend.valid
                text: qsTr("NO SIGNAL")
                color: Material.color(Material.Red)
                font.bold: true
            }

            Text {
                anchors.centerIn: parent
                visible: viewer.loadedCount === 0
                text: robotScene.linkCount > 0 ? qsTr("Loading robot links…")
                                               : robotScene.status
                color: "#c8a5ff"
                wrapMode: Text.WordWrap
                width: parent.width * 0.72
                horizontalAlignment: Text.AlignHCenter
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 20

            Label {
                text: qsTr("FANUC  %1, %2, %3")
                    .arg(backend.fanucPos.x.toFixed(1))
                    .arg(backend.fanucPos.y.toFixed(1))
                    .arg(backend.fanucPos.z.toFixed(1))
                opacity: backend.stale ? 0.4 : 1.0
                font.pixelSize: 12
            }

            Label {
                text: qsTr("Camera  %1, %2, %3")
                    .arg(backend.cameraPos.x.toFixed(1))
                    .arg(backend.cameraPos.y.toFixed(1))
                    .arg(backend.cameraPos.z.toFixed(1))
                opacity: (backend.stale || !backend.valid) ? 0.4 : 1.0
                font.pixelSize: 12
            }

            Item { Layout.fillWidth: true }
        }
    }
}
