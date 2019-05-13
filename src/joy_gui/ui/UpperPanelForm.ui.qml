import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

Page {

    implicitWidth: 640
    implicitHeight: 460

    header: Label {
        id: page_label
        text: qsTr("Joystick output")
        font.pixelSize: Qt.application.font.pixelSize * 1.5
        padding: 10
    }

    id: upperPanelPage

    property alias spinboxLinear: spinboxLinear
    property alias spinboxAngular: spinboxAngular
    property alias vbars: vbars
    property alias reloadButton: reloadButton

    Grid {
        id: grid1
        spacing: 10

        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.fill: parent

        rows: 2
        columns: 2

        /* Velocity bars */
        GroupBox {

            id: vbarsGroupBox
            title: "Velocity command"

            VelocityBar6D {
                id: vbars
            }
        }

        /* Mistery item */
        GroupBox {

            title: "Mistery box"

            Image {
                id: wipImage
                sourceSize.height: 200
                sourceSize.width: 280
                source: "../pics/worker.svg"
            }
        }

        /* Speed limits */
        GroupBox {

            title: "Speed limits"
            width: vbarsGroupBox.width

            GridLayout {
                id: grid
                columnSpacing: 13
                rows: 2
                columns: 2

                Label {
                    id: label
                    text: "Max linear speed"
                }

                DoubleSpinbox2 {
                    id: spinboxLinear
                    wrap: false
                    editable: false
                    from: 0.0
                    to: 1.0
                    stepSize: 0.1
                }

                Label {
                    id: label1
                    text: "Max angular speed"
                }

                DoubleSpinbox2 {
                    id: spinboxAngular
                    editable: false
                    wrap: false
                    from: 0.0
                    to: 1.0
                    stepSize: 0.1
                }
            }
        }

        DelayButton {
            id: reloadButton
            text: "Restart"
            delay: 3000
        }
    }
}
