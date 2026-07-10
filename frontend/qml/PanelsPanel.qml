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
            text: qsTr("Panels")
            font.bold: true
            font.pixelSize: 18
        }

        Label {
            Layout.fillWidth: true
            text: qsTr("error: %1 mm").arg(backend.errorMm.toFixed(2))
            font.pixelSize: 16
        }

        Label {
            Layout.fillWidth: true
            text: backend.stale ? qsTr("NO DATA (stale)")
                                 : (backend.valid ? qsTr("live") : qsTr("tracking invalid"))
            color: backend.stale ? Material.color(Material.Grey)
                                  : (backend.valid ? Material.color(Material.Green) : Material.color(Material.Red))
            font.bold: true
        }

        Label {
            Layout.fillWidth: true
            text: qsTr("Δx %1  Δy %2  Δz %3 mm")
                .arg(backend.errorXMm.toFixed(2))
                .arg(backend.errorYMm.toFixed(2))
                .arg(backend.errorZMm.toFixed(2))
            font.pixelSize: 12
            opacity: 0.7
        }

        Label {
            Layout.fillWidth: true
            text: qsTr("error history (mm)")
            font.pixelSize: 12
            opacity: 0.6
        }

        MuSparkline {
            Layout.fillWidth: true
            Layout.preferredHeight: 80
            values: backend.errorHistory
        }

        Item { Layout.fillHeight: true }
    }
}
