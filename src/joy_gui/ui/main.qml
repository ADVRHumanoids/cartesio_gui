import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

ApplicationWindow {

    visible: true
    width: 660
    height: 760
    title: qsTr("CartesI/O Joystick GUI")

    /* C++ backend */
    BackEndComponent
    {
        id: backend
    }

    /* Timer to update data from CI */
    Timer
    {
        interval: 16
        running: true
        repeat: true

        onTriggered:
        {
            backend.updateCiData()
        }
    }



    /* Main layout */
    ColumnLayout {

        anchors.fill: parent

        UpperPanel
        {
            id: upperPanel
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
        }

        LowerPanel
        {
            id: lowerPanel
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
        }

    }

    /* Tab bar for task selection */
    footer: TaskTabBar {
        id: taskTabBar
    }
}



/*##^## Designer {
    D{i:0;height:700;width:640}
}
 ##^##*/
