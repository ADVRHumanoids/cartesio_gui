import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

Page {

    header: Label {
        id: page_label
        text: qsTr("Joystick output")
        font.pixelSize: Qt.application.font.pixelSize * 1.5
        padding: 10
    }

    id: frame
    width: 200
    height: 200
    Layout.preferredHeight: 310
    Layout.fillHeight: false
    Layout.fillWidth: true
    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

    GroupBox {

        id: groupBox

        x: 20
        y: 0
        width: 295
        height: 263
        contentWidth: vbars.width
        contentHeight: vbars.height
        title: "Output level"

        VelocityBar6D {
            id: vbars
            anchors.left: parent.left
            anchors.leftMargin: 0
            anchors.top: parent.top
            anchors.topMargin: 0
        }
    }
}
