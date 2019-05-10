import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

ApplicationWindow {
    visible: true
    width: 640
    height: 700
    title: qsTr("CartesI/O Joystick GUI")


    JoyGuiBackEnd
    {
        id: backend

        property bool send_service: false

        function updateGui()
        {
            print("Updating gui..")

            /* Set swipe tab according to active task */
            tabBar.setCurrentIndex(activeTaskIndex())

            /* Disable sending requests to the joy spawner */
            send_service = false

            /* Update task baselink */
            var baselink = backend.getBaseLink()
            var baselink_idx = swipeView.currentItem.comboBox.find(baselink)
            swipeView.currentItem.comboBox.currentIndex = baselink_idx

            /* Update safety limits */
            var max_lin = backend.getMaxLinearSpeed()
            swipeView.currentItem.spinboxLinear.value = max_lin

            var max_ang = backend.getMaxAngularSpeed()
            swipeView.currentItem.spinboxAngular.value = max_ang

            /* Update enabled axes */
            vbars.set_enabled_axis(backend.getEnabledAxis())

            /* Update control mode */
            var ctrl = backend.getControlMode(backend.activeTask)

            if(ctrl === JoyGuiBackEnd.ControlMode.Velocity)
            {
                swipeView.currentItem.taskSwitch.checked = true
            }
            else
            {
                swipeView.currentItem.taskSwitch.checked = false
            }

            /* Update reference frame */
            var ref_frame = backend.getRefFrame()

            if(ref_frame === "")
            {
                swipeView.currentItem.radioButtonGlobal.checked = true
            }
            else if(ref_frame === "base_link")
            {
                swipeView.currentItem.radioButtonBaseLink.checked = true
            }
            else if(ref_frame === backend.activeTask)
            {
                swipeView.currentItem.radioButtonLocal.checked = true
            }
            else
            {
                print("Error: reference frame not valid -> " + ref_frame)
            }

            /* Re-enable services */
            send_service = true
        }

        Component.onCompleted:
        {

        }

        onActiveTaskChanged:
        {
            // tabBar.setCurrentIndex(activeTaskIndex())
            // updateGui()
        }

        onJoyStatusReceived:
        {
            updateGui()
        }



    }


    ColumnLayout {
        anchors.fill: parent

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

            GroupBox
            {
                id: groupBox

                x: 20
                y: 0
                width: 295
                height: 263
                contentWidth: vbars.width
                contentHeight: vbars.height
                title: "Output level"


                VelocityBar6D
                {
                    id: vbars
                    anchors.left: parent.left
                    anchors.leftMargin: 0
                    anchors.top: parent.top
                    anchors.topMargin: 0

                }

            }
        }

        SwipeView {

            id: swipeView
            Layout.fillHeight: true
            Layout.fillWidth: true
            currentIndex: tabBar.currentIndex

            Component.onCompleted:
            {
                var tasks = backend.getTasks()

                for(var i = 0; i < tasks.length; i++)
                {
                    var comp = Qt.createComponent("TaskPage.qml")
                    var page = comp.createObject(swipeView)
                    page.page_label.text = tasks[i]

                    addItem(page)
                }

            }

            onCurrentIndexChanged:
            {
                if(backend.send_service)
                {
                    backend.activeTask = swipeView.currentItem.page_label.text
                }

            }





        }


    }

    footer: TabBar {
        id: tabBar
        currentIndex: swipeView.currentIndex

        Component
        {
            id: tab_button_comp
            TabButton { font.pointSize: 9}
        }


        Component.onCompleted:
        {
            var tasks = backend.getTasks()

            for(var i = 0; i < tasks.length; i++)
            {
                var button = tab_button_comp.createObject(tabBar, {text: tasks[i]})
                addItem(button)
            }
        }


    }
}




















/*##^## Designer {
    D{i:6;anchors_x:20}
}
 ##^##*/
