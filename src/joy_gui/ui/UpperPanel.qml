import QtQuick 2.4

UpperPanelForm {

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

    reloadButton.onActivated:
    {
        backend.restart_process()
    }

    resetViewButton.onReleased:
    {
        viewer3d.resetCamera()
    }



}





/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
