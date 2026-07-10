import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import Mu.Material

MuApp {
    id: window
    width: 1000
    height: 650
    title: qsTr("Camera Tracking — Hand Position Validation")
    header: MuAppBar { title: window.title }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: bannerLabel.implicitHeight + 12
            visible: backend.fanucIsStub
            color: "#c0392b"

            Label {
                id: bannerLabel
                anchors.centerIn: parent
                text: qsTr("FANUC DATA IS SIMULATED (stub mode) — not a real controller link")
                color: "white"
                font.bold: true
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 8

            ViewportPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 3
            }

            PanelsPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 1
            }
        }
    }
}
