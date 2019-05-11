import QtQuick 2.4

TaskPageForm {

    Component.onCompleted:
    {
        comboBox.model = backend.getLinks()
    }

    taskSwitch.onCheckedChanged:
    {
        if(taskSwitch.checked)
        {
            taskSwitch.text = "Enabled"
            if(backend.send_service)
            {
                backend.enableVelocityControl(true)
            }
        }
        else
        {
            taskSwitch.text = "Disabled"
            if(backend.send_service)
            {
                backend.enableVelocityControl(false)
            }
        }
    }



    radioButtonGlobal.onToggled:
    {
        if(backend.send_service)
        {
            backend.setRefFrame("global");
        }
    }

    radioButtonLocal.onToggled:
    {
        if(backend.send_service)
        {
            backend.setRefFrame("local");
        }
    }

    radioButtonBaseLink.onToggled:
    {
        if(backend.send_service)
        {
            backend.setRefFrame("base_link");
        }
    }

    comboBox.onActivated:
    {
        if(backend.send_service)
        {
            backend.setBaseLink(comboBox.currentText)
        }
    }

}
