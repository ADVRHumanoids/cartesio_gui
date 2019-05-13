import QtQuick 2.4
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

TabBar {
    id: tabBar
    currentIndex: lowerPanel.swipeView.currentIndex

    TabButton {
        text: "TaskName"
    }

    Component
    {
        id: tabButtonComponent
        TabButton { font.pointSize: 9 }
    }

    function loadTabButtons()
    {
        while(tabBar.count > 0)
        {
            tabBar.removeItem(tabBar.currentItem)
        }

        console.info("Loading tab bar buttons..")

        var tasks = backend.getTasks()

        for(var i = 0; i < tasks.length; i++)
        {
            console.info("Adding task '" + tasks[i] + "'")

            var button = tabButtonComponent.createObject(tabBar, {text: tasks[i]})
            tabBar.addItem(button)
        }
    }

    Component.onCompleted: loadTabButtons()


}

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
