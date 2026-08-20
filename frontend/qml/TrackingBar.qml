pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import CameraTracking
ThemedCard {
    id: root
    clip: true
    readonly property real geometryWarnMm: 2.0

    readonly property int headlineWidth: 190
    readonly property int thetaWidth: 120
    readonly property int statWidth: 170

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            StatusChip {
                text: backend.stale ? qsTr("BACKEND SILENT") : qsTr("tracking")
                tone: backend.stale ? Theme.critical : Theme.ok
            }
            StatusChip {
                text: backend.loggingActive ? qsTr("LOGGING") : qsTr("not logging")
                tone: backend.loggingActive ? Theme.ok : Theme.caution
            }
            StatusChip {
                visible: backend.reviewAccepted
                text: qsTr("placement accepted")
                tone: Theme.neutral
            }
            Item { Layout.fillWidth: true }
            Label {
                text: backend.anchorFrame
                      ? qsTr("%1 · %2").arg(backend.anchorFrame).arg(backend.anchorStatus)
                      : ""
                color: backend.anchorStatus === "anchored" ? Theme.textSecondary : Theme.caution
                font.pixelSize: 11
            }
        }

        ScrollView {
            id: scroll
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: scroll.availableWidth
                spacing: 6

                Label {
                    Layout.fillWidth: true
                    text: qsTr("HAND JOINTS · measured by camera")
                    color: Theme.textMuted
                    font.pixelSize: 10
                    font.letterSpacing: 1
                }

                Label {
                    Layout.fillWidth: true
                    visible: backend.jointStatus !== ""
                    text: backend.jointStatus
                    color: Theme.caution
                    wrapMode: Text.WordWrap
                    font.pixelSize: 11
                }

                Repeater {
                    model: backend.jointRows

                    delegate: Rectangle {
                        required property var modelData

                        Layout.fillWidth: true
                        implicitHeight: jointCol.implicitHeight + 16
                        radius: Theme.radiusSmall
                        color: Theme.surfaceInset
                        border.width: 1
                        border.color: modelData.estimated ? Theme.outline : Theme.seBrown4

                        ColumnLayout {
                            id: jointCol
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 10

                                Label {
                                    text: modelData.jointId
                                    color: Theme.textSecondary
                                    font.pixelSize: 12
                                }
                                Item { Layout.fillWidth: true }
                                Label {
                                    Layout.preferredWidth: root.thetaWidth
                                    horizontalAlignment: Text.AlignRight
                                    text: qsTr("%1°").arg(modelData.thetaDeg.toFixed(2))
                                    color: modelData.estimated ? Theme.textPrimary : Theme.textMuted
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 20
                                }
                                StatusChip {
                                    visible: modelData.clamped
                                    text: qsTr("CLAMPED")
                                    tone: Theme.critical
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 12

                                Label {

                                    Layout.preferredWidth: root.statWidth
                                    elide: Text.ElideRight
                                    text: qsTr("residual %1 mm / %2°")
                                            .arg(modelData.residualMm.toFixed(1))
                                            .arg(modelData.residualDeg.toFixed(3))
                                    color: Theme.textSecondary
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 11
                                }
                                Label {
                                    
                                    Layout.preferredWidth: root.statWidth + 40
                                    elide: Text.ElideRight
                                    text: qsTr("radial %1 mm · axial %2 mm")
                                            .arg(modelData.radialErrorMm.toFixed(2))
                                            .arg(modelData.axialErrorMm.toFixed(2))
                                    color: (Math.abs(modelData.radialErrorMm) > root.geometryWarnMm
                                            || Math.abs(modelData.axialErrorMm) > root.geometryWarnMm)
                                           ? Theme.caution : Theme.textSecondary
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 11
                                }
                                Label {
                                    Layout.preferredWidth: root.statWidth - 40
                                    elide: Text.ElideRight
                                    text: qsTr("confidence %1").arg(modelData.confidence.toFixed(3))
                                    color: Theme.textSecondary
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 11
                                }
                                Item { Layout.fillWidth: true }
                                Label {
                                    text: qsTr("limits %1° … %2°")
                                            .arg(modelData.lowerDeg.toFixed(1))
                                            .arg(modelData.upperDeg.toFixed(1))
                                    color: Theme.textMuted
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 10
                                }
                            }

                            Label {
                                Layout.fillWidth: true
                                visible: !modelData.estimated
                                text: modelData.skipReason
                                color: Theme.caution
                                wrapMode: Text.WordWrap
                                font.pixelSize: 10
                            }

                            Label {
                                Layout.fillWidth: true
                                text: modelData.reportedValid
                                      ? qsTr("controller says %1° · error %2°")
                                            .arg(modelData.reportedDeg.toFixed(2))
                                            .arg(modelData.errorDeg.toFixed(2))
                                      : modelData.reportedStatus
                                color: modelData.reportedValid ? Theme.textSecondary : Theme.textMuted
                                wrapMode: Text.WordWrap
                                font.pixelSize: 10
                            }
                        }
                    }
                }

                Label {
                    Layout.fillWidth: true
                    visible: backend.jointRows.length > 0
                    text: qsTr("PLACEMENTS · camera vs robot")
                    color: Theme.textMuted
                    font.pixelSize: 10
                    font.letterSpacing: 1
                    Layout.topMargin: 6
                }

                Label {
                    Layout.fillWidth: true
                    visible: backend.trackingRows.length === 0
                   
                    text: qsTr("No continuously-tracked comparison in this scene. The latched "
                               + "placements were settled at the review gate; add a "
                               + "continuously-tracked object to see a live delta here.")
                    color: Theme.textSecondary
                    wrapMode: Text.WordWrap
                    font.pixelSize: 11
                }

                Repeater {
                    model: backend.trackingRows

                    delegate: Rectangle {
                        required property var modelData

                        Layout.fillWidth: true
                        implicitHeight: rowCol.implicitHeight + 16
                        radius: Theme.radiusSmall
                        color: Theme.surfaceInset
                        border.width: 1
                        border.color: modelData.valid ? Theme.outline : Theme.seBrown4

                        ColumnLayout {
                            id: rowCol
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 10

                                Label {
                                    text: modelData.objectId
                                    color: Theme.textPrimary
                                    font.bold: true
                                    font.pixelSize: 14
                                }
                                Label {
                                    Layout.preferredWidth: root.statWidth
                                    elide: Text.ElideRight
                                    visible: modelData.valid && modelData.timeGapS > 0
                                    text: qsTr("matched %1 ms apart")
                                          .arg((modelData.timeGapS * 1000).toFixed(1))
                                    color: Theme.textMuted
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 10
                                }
                                Item { Layout.fillWidth: true }
                                Label {
                                    Layout.preferredWidth: root.headlineWidth
                                    horizontalAlignment: Text.AlignRight
                                    text: modelData.valid
                                          ? qsTr("%1 mm   ∠ %2°")
                                                .arg(modelData.distanceMm.toFixed(2))
                                                .arg(modelData.angleDeg.toFixed(3))
                                          : qsTr("no reading")
                                    color: modelData.valid ? Theme.ok : Theme.neutral
                                    font.family: Theme.numericFamily
                                    font.bold: true
                                    font.pixelSize: 17
                                }
                            }

                            GridLayout {
                                Layout.fillWidth: true
                                columns: 3
                                columnSpacing: 16
                                rowSpacing: 1

                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    text: qsTr("OptiTrack · %1").arg(modelData.aId)
                                    color: Theme.textMuted
                                    font.pixelSize: 9
                                    elide: Text.ElideRight
                                }
                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    text: qsTr("Robot · %1").arg(modelData.bId)
                                    color: Theme.textMuted
                                    font.pixelSize: 9
                                    elide: Text.ElideRight
                                }
                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    text: qsTr("Discrepancy")
                                    color: Theme.textMuted
                                    font.pixelSize: 9
                                    elide: Text.ElideRight
                                }

                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    elide: Text.ElideRight
                                    text: modelData.aValid
                                          ? qsTr("%1  %2  %3 mm")
                                                .arg(modelData.aXMm.toFixed(1))
                                                .arg(modelData.aYMm.toFixed(1))
                                                .arg(modelData.aZMm.toFixed(1))
                                          : qsTr("not placed")
                                    color: modelData.aValid ? Theme.textPrimary : Theme.caution
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 12
                                }
                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    elide: Text.ElideRight
                                    text: modelData.bValid
                                          ? qsTr("%1  %2  %3 mm")
                                                .arg(modelData.bXMm.toFixed(1))
                                                .arg(modelData.bYMm.toFixed(1))
                                                .arg(modelData.bZMm.toFixed(1))
                                          : qsTr("not placed")
                                    color: modelData.bValid ? Theme.textPrimary : Theme.caution
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 12
                                }
                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    elide: Text.ElideRight
                                    text: modelData.valid
                                          ? qsTr("%1  %2  %3 mm")
                                                .arg(modelData.dxMm.toFixed(2))
                                                .arg(modelData.dyMm.toFixed(2))
                                                .arg(modelData.dzMm.toFixed(2))
                                          : qsTr("—")
                                    color: Theme.textPrimary
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 12
                                }

                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    elide: Text.ElideRight
                                    text: modelData.aValid
                                          ? qsTr("%1  %2  %3°")
                                                .arg(modelData.aRollDeg.toFixed(2))
                                                .arg(modelData.aPitchDeg.toFixed(2))
                                                .arg(modelData.aYawDeg.toFixed(2))
                                          : ""
                                    color: Theme.textSecondary
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 11
                                }
                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    elide: Text.ElideRight
                                    text: modelData.bValid
                                          ? qsTr("%1  %2  %3°")
                                                .arg(modelData.bRollDeg.toFixed(2))
                                                .arg(modelData.bPitchDeg.toFixed(2))
                                                .arg(modelData.bYawDeg.toFixed(2))
                                          : ""
                                    color: Theme.textSecondary
                                    font.family: Theme.numericFamily
                                    font.pixelSize: 11
                                }
                                Label {
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 1
                                    text: qsTr("x y z · euler XYZ")
                                    color: Theme.textMuted
                                    font.pixelSize: 9
                                    elide: Text.ElideRight
                                }
                            }

                            Label {
                                Layout.fillWidth: true
                                visible: !modelData.valid
                                text: modelData.invalidReason
                                color: Theme.caution
                                wrapMode: Text.WordWrap
                                font.pixelSize: 10
                            }
                        }
                    }
                }
            }
        }
    }
}
