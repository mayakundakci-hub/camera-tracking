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

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 80
            radius: 8
            color: "#151225"
            border.color: "#6b2696"
            border.width: 1
            clip: true

            Canvas {
                id: spark
                anchors.fill: parent
                anchors.margins: 6

                property var values: backend.errorHistory
                onValuesChanged: requestPaint()

                onPaint: {
                    var ctx = getContext("2d")
                    ctx.reset()

                    var n = values ? values.length : 0
                    if (n < 2)
                        return

                    var minV = values[0], maxV = values[0]
                    for (var i = 1; i < n; ++i) {
                        if (values[i] < minV) minV = values[i]
                        if (values[i] > maxV) maxV = values[i]
                    }
                    var range = maxV - minV
                    if (range < 1e-9) range = 1

                    ctx.strokeStyle = "#c8a5ff"
                    ctx.lineWidth = 1.5
                    ctx.beginPath()
                    for (var j = 0; j < n; ++j) {
                        var x = (j / (n - 1)) * width
                        var y = height - ((values[j] - minV) / range) * height
                        if (j === 0) ctx.moveTo(x, y)
                        else ctx.lineTo(x, y)
                    }
                    ctx.stroke()
                }
            }
        }

        Item { Layout.fillHeight: true }
    }
}
