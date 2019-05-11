import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

Item {

    id: lowerPanelPage

    property alias swipeView: swipeView
    implicitHeight: 280

    SwipeView {

        id: swipeView
        currentIndex: taskTabBar.currentIndex

        TaskPage {
        }
    }
}




/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
