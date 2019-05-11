import QtQuick 2.4

Item {
    id: element

    width: 40
    height: 400
    property double height_ratio: 0.25
    property double y_ratio: 0.5
    property alias rectangle: rectangle

    Rectangle {
        id: rectangle
        x: 0
        y: parent.height * y_ratio
        width: parent.width * 0.8
        height: parent.height * height_ratio
        color: "#c9c9c9"
        anchors.horizontalCenter: parent.horizontalCenter
        border.color: "#00000000"
    }

    Rectangle {
        id: border_rect
        color: "#00000000"
        border.width: 1
        z: 2
        anchors.fill: parent
    }

    Rectangle {
        id: rect_background
        x: 0
        y: 0
        width: parent.width
        height: parent.height
        color: "#e9e9e9"
        z: -4
    }
}




/*##^## Designer {
    D{i:2;anchors_height:400;anchors_width:40;anchors_x:0;anchors_y:0}
}
 ##^##*/
