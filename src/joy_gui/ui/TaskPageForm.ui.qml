import QtQuick 2.12
import QtQuick.Controls 2.5
import QtQuick.Controls 2.3

Page {

    width: 600
    height: 400
    property alias radioButtonBaseLink: radioButtonBaseLink
    property alias radioButtonLocal: radioButtonLocal
    property alias radioButtonGlobal: radioButtonGlobal
    property alias spinboxAngular: spinboxAngular
    property alias spinboxLinear: spinboxLinear
    property alias comboBox: comboBox
    property alias taskSwitch: taskSwitch
    property alias page_label: page_label

    header: Label {
        id: page_label
        text: qsTr("Page 1")
        font.pixelSize: Qt.application.font.pixelSize * 2
        padding: 10
    }

    GroupBox {
        id: groupBox
        x: 23
        y: 1
        width: 200
        height: 172
        title: qsTr("Reference frame")

        RadioButton {
            id: radioButtonGlobal
            x: 0
            y: 0
            text: qsTr("Global frame")
            checked: true
        }

        RadioButton {
            id: radioButtonLocal
            x: 0
            y: 46
            text: qsTr("Local frame")
        }

        RadioButton {
            id: radioButtonBaseLink
            x: 0
            y: 92
            text: qsTr("Base link")
        }
    }

    GroupBox {
        id: groupBox1
        x: 247
        y: 184
        width: 200
        height: 88
        title: qsTr("Base link ")

        ComboBox {
            id: comboBox
            x: 0
            y: 0
            width: 176
            height: 40
        }
    }

    GroupBox {
        id: groupBox2
        x: 247
        y: 0
        width: 329
        height: 146
        title: qsTr("Speed limits")

        Label {
            id: label
            x: 9
            y: 12
            text: qsTr("Max linear speed")
        }

        DoubleSpinbox2 {
            id: spinboxLinear
            wrap: false
            editable: false
            // anchors.rightMargin: 150
            anchors.right: parent.right
            from: 0.0
            to: 1.0
            stepSize: 0.1
        }

        Label {
            id: label1
            x: 9
            y: 69
            text: qsTr("Max angular speed")
        }

        DoubleSpinbox2 {
            id: spinboxAngular
            x: 2
            y: 57
            editable: false
            wrap: false
            anchors.right: parent.right
            // anchors.rightMargin: 150
            from: 0.0
            to: 1.0
            stepSize: 0.1
        }
    }

    GroupBox {
        id: groupBox3
        x: 23
        y: 184
        width: 200
        height: 88
        title: qsTr("Enable/Disable task")

        Switch {
            id: taskSwitch
            x: 0
            y: 0
            text: qsTr("Disabled")
        }
    }
}




/*##^## Designer {
    D{i:9;anchors_x:7;anchors_y:"-8"}D{i:11;anchors_x:7;anchors_y:"-8"}
}
 ##^##*/
