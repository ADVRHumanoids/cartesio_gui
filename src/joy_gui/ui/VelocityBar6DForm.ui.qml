import QtQuick 2.4

Item {

    id: element
    width: 280
    height: 220
    property alias bar_yaw: bar_yaw
    property alias bar_p: bar_p
    property alias bar_r: bar_r
    property alias bar_z: bar_z
    property alias bar_y: bar_y
    property alias bar_x: bar_x

    Row {
        anchors.top: parent.top
        anchors.topMargin: -5

        anchors.left: parent.left
        anchors.leftMargin: 2
        spacing: 6

        VelocityBar {
            id: bar_x
            label.text: "X"
            index: 0
        }

        VelocityBar {
            id: bar_y
            label.text: "Y"
            index: 1
        }

        VelocityBar {
            id: bar_z
            label.text: "Z"
            index: 2
        }

        VelocityBar {
            id: bar_r
            label.text: "R"
            index: 3
        }

        VelocityBar {
            id: bar_p
            label.text: "P"
            index: 4
        }

        VelocityBar {
            id: bar_yaw
            label.text: "Y"
            index: 5
        }
    }
}
