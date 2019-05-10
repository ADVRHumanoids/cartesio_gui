import QtQuick 2.4
import joygui.backend 1.0

VelocityBar6DForm {



    function set_max_linear_speed(vel)
    {
        bar_x.signedBar.from = -vel
        bar_y.signedBar.from = -vel
        bar_z.signedBar.from = -vel

        bar_x.signedBar.to = vel
        bar_y.signedBar.to = vel
        bar_z.signedBar.to = vel

    }

    function set_max_angular_speed(vel)
    {
        bar_r.signedBar.from = -vel
        bar_p.signedBar.from = -vel
        bar_yaw.signedBar.from = -vel

        bar_r.signedBar.to = vel
        bar_p.signedBar.to = vel
        bar_yaw.signedBar.to = vel
    }

    function set_enabled_axis(flags)
    {
        bar_x.checkBox.checked = flags[0]
        bar_y.checkBox.checked = flags[1]
        bar_z.checkBox.checked = flags[2]
        bar_r.checkBox.checked = flags[3]
        bar_p.checkBox.checked = flags[4]
        bar_yaw.checkBox.checked = flags[5]
    }


    Timer
    {
        interval: 16
        running: true
        repeat: true

        onTriggered:
        {
            var twist = backend.getTwist()

            bar_x.signedBar  .value = twist[0]
            bar_y.signedBar  .value = twist[1]
            bar_z.signedBar  .value = twist[2]
            bar_r.signedBar  .value = twist[3]
            bar_p.signedBar  .value = twist[4]
            bar_yaw.signedBar.value = twist[5]
        }
    }

    Timer
    {
        id: slowtimer

        interval: 1000
        running: true
        repeat: false

        onTriggered:
        {
            backend.updateGui()
        }
    }


}
