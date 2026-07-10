import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Mu.Material

MuCard {
    id: root

    ColumnLayout {
        anchors.fill: parent
        spacing: 12

        Label {
            Layout.fillWidth: true
            text: qsTr("Viewport")
            font.bold: true
            font.pixelSize: 18
        }

        Label {
            Layout.fillWidth: true
            visible: backend.stale || !backend.valid
            text: qsTr("NO SIGNAL")
            color: Material.color(Material.Red)
            font.bold: true
        }

        Label {
            Layout.fillWidth: true
            text: qsTr("FANUC:  %1, %2, %3")
                .arg(backend.fanucPos.x.toFixed(3))
                .arg(backend.fanucPos.y.toFixed(3))
                .arg(backend.fanucPos.z.toFixed(3))
            opacity: backend.stale ? 0.4 : 1.0
        }

        Label {
            Layout.fillWidth: true
            text: qsTr("Camera: %1, %2, %3")
                .arg(backend.cameraPos.x.toFixed(3))
                .arg(backend.cameraPos.y.toFixed(3))
                .arg(backend.cameraPos.z.toFixed(3))
            opacity: (backend.stale || !backend.valid) ? 0.4 : 1.0
        }

        Item { Layout.fillHeight: true }

        Label {
            // TODO: load environment STLs / URDF here and render the rotor +
            // dual markers. Now that the frontend is QML, Qt Quick 3D is the
            // natural fit (vs. Qt3D / QOpenGLWidget considered previously).
            // TODO: marker convention. TODO: interpolate marker motion
            // between samples instead of snapping to the latest sample.
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
            text: qsTr("3D rendering not implemented yet")
            font.italic: true
            opacity: 0.6
        }
    }
}
