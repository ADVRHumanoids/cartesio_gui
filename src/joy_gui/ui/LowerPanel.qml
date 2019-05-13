import QtQuick 2.4
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

LowerPanelForm {

    function loadSwipeViewPages()
    {
        while(swipeView.count > 0)
        {
            swipeView.removeItem(swipeView.currentItem)
        }

        console.info("Loading swipe view pages..")

        var tasks = backend.getTasks()

        for (var i = 0; i < tasks.length; i++)
        {
            console.info("Adding task '" + tasks[i] + "'")

            var comp = Qt.createComponent("TaskPage.qml")
            var page = comp.createObject(swipeView)
            page.pageLabel.text = tasks[i]

            swipeView.addItem(page)
        }
    }


    Component.onCompleted:
    {
        loadSwipeViewPages()
    }

    swipeView.onCurrentIndexChanged:
    {
        if (backend.send_service)
        {
            backend.activeTask = swipeView.currentItem.pageLabel.text
        }
    }



}

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
