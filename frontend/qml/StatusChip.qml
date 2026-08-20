import QtQuick
import QtQuick.Controls
import CameraTracking


Rectangle {
    id: root

    property alias text: label.text
    property color tone: Theme.neutral

    readonly property color textColor: Theme.textOn(root.tone)

    implicitWidth: label.implicitWidth + 20
    implicitHeight: label.implicitHeight + 8
    radius: height / 2
    color: root.tone

    Label {
        id: label
        anchors.centerIn: parent
        color: root.textColor
        font.bold: true
        font.pixelSize: 11
    }
}
