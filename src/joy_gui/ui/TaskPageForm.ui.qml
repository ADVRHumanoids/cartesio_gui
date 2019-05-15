import QtQuick 2.12
import QtQuick.Controls 2.5
import QtQuick.Controls 2.3
import QtQuick.Layouts 1.3

Page {

    implicitWidth: 640
    implicitHeight: 240

    property alias radioButtonBaseLink: radioButtonBaseLink
    property alias radioButtonLocal: radioButtonLocal
    property alias radioButtonGlobal: radioButtonGlobal
    property alias comboBox: comboBox
    property alias taskSwitch: taskSwitch
    property alias pageLabel: pageLabel

    /* Header shows task name */
    header: Label {
        id: pageLabel
        text: "PageName"
        font.pixelSize: Qt.application.font.pixelSize * 2
        padding: 10
    }

    GridLayout {
        anchors.rightMargin: 10
        anchors.leftMargin: 10
        anchors.fill: parent
        rows: 2
        columns: 2



        /* Base link choice */
        GroupBox {

            id: baseLinkGroupBox
            Layout.fillWidth: false
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            title: "Base link "

            ComboBox {
                id: comboBox
            }
        }

        /* Reference frame choice */
        GroupBox {

            id: refFrameGroupBox
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            title: "Reference frame"
            Layout.rowSpan: 2

            Column {

                RadioButton {
                    id: radioButtonGlobal
                    text: "Global frame"
                    checked: true
                }

                RadioButton {
                    id: radioButtonLocal
                    text: "Local frame"
                }

                RadioButton {
                    id: radioButtonBaseLink
                    text: "Base link"
                }
            }
        }

        /* Enable/disable joy control */
        GroupBox {
            id: enableCtrlGroupBox
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            title: "Enable/Disable task"

            Switch {
                id: taskSwitch
                text: "Disabled"
            }
        }
    }
}
