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

    spinboxLinear.onValueChanged:
    {
        vbars.set_max_linear_speed(spinboxLinear.value)

        if(backend.send_service)
        {
            backend.setMaxLinearSpeed(spinboxLinear.value)
        }
    }

    spinboxAngular.onValueChanged:
    {
        vbars.set_max_angular_speed(spinboxAngular.value)

        if(backend.send_service)
        {
            backend.setMaxAngularSpeed(spinboxAngular.value)
        }
    }

    radioButtonGlobal.onCheckedChanged:
    {
        if(backend.send_service)
        {
            backend.setRefFrame("global");
        }
    }

    radioButtonLocal.onCheckedChanged:
    {
        if(backend.send_service)
        {
            backend.setRefFrame("local");
        }
    }

    radioButtonBaseLink.onCheckedChanged:
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
