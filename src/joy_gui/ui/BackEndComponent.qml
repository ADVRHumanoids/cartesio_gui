import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0

import joygui.backend 1.0

JoyGuiBackEnd
{
    id: backend

    /* Variable used to inhibit service calls */
    property bool send_service: false

    /* Function to update gui whenever a new joystick status is received */
    function updateGui()
    {
        print("Updating gui..")

        /* Disable sending requests to the joy spawner */
        send_service = false

        /* Set swipe tab according to active task */
        taskTabBar.setCurrentIndex(activeTaskIndex())
        upperPanel.viewer3d.setTaskName(backend.activeTask)

        /* Update task baselink */
        var baselink = backend.getBaseLink()
        var baselink_idx = lowerPanel.swipeView.currentItem.comboBox.find(baselink)
        lowerPanel.swipeView.currentItem.comboBox.currentIndex = baselink_idx

        /* Update safety limits */
        var max_lin = backend.getMaxLinearSpeed()
        upperPanel.spinboxLinear.value = max_lin

        var max_ang = backend.getMaxAngularSpeed()
        upperPanel.spinboxAngular.value = max_ang

        /* Update enabled axes */
        upperPanel.vbars.set_enabled_axis(backend.getEnabledAxis())

        /* Update control mode */
        var ctrl = backend.getControlMode(backend.activeTask)

        if(ctrl === JoyGuiBackEnd.ControlMode.Velocity)
        {
            lowerPanel.swipeView.currentItem.taskSwitch.checked = true
        }
        else
        {
            lowerPanel.swipeView.currentItem.taskSwitch.checked = false
        }

        /* Update reference frame */
        var ref_frame = backend.getRefFrame()

        if(ref_frame === "")
        {
            lowerPanel.swipeView.currentItem.radioButtonGlobal.checked = true
            upperPanel.viewer3d.setBaseFrame("global")
        }
        else if(ref_frame === "base_link")
        {
            lowerPanel.swipeView.currentItem.radioButtonBaseLink.checked = true
            upperPanel.viewer3d.setBaseFrame("base_link")
        }
        else if(ref_frame === backend.activeTask)
        {
            lowerPanel.swipeView.currentItem.radioButtonLocal.checked = true
            upperPanel.viewer3d.setBaseFrame("local")
        }
        else
        {
            print("Error: reference frame not valid -> " + ref_frame)
        }


        /* Re-enable services */
        send_service = true
    }

    function updateCiData()
    {
        send_service = false

        /* Update control mode */
        var ctrl = backend.getControlMode(backend.activeTask)

        if(ctrl === JoyGuiBackEnd.ControlMode.Velocity)
        {
            lowerPanel.swipeView.currentItem.taskSwitch.checked = true
        }
        else
        {
            lowerPanel.swipeView.currentItem.taskSwitch.checked = false
        }

        /* Update reference frame */
        var ref_frame = backend.getRefFrame()

        if(ref_frame === "")
        {
            lowerPanel.swipeView.currentItem.radioButtonGlobal.checked = true
            upperPanel.viewer3d.setBaseFrame("global")
        }
        else if(ref_frame === "base_link")
        {
            lowerPanel.swipeView.currentItem.radioButtonBaseLink.checked = true
            upperPanel.viewer3d.setBaseFrame("base_link")
        }
        else if(ref_frame === backend.activeTask)
        {
            lowerPanel.swipeView.currentItem.radioButtonLocal.checked = true
            upperPanel.viewer3d.setBaseFrame("local")
        }
        else
        {
            print("Error: reference frame not valid -> " + ref_frame)
        }

        send_service = true
    }

    Component.onCompleted:
    {

    }

    onActiveTaskChanged:
    {
        updateGui()
    }

    onJoyStatusReceived:
    {
        updateGui()
    }


}

