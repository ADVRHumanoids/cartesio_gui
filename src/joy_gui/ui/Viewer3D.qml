
import QtQuick 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import QtQuick.Scene3D 2.0
import Viewer3D.Utils3D 1.0
import QtQuick.Controls 2.5

Item
{
    id: component

    implicitHeight: 200
    implicitWidth: 300

    function setTaskName(task)
    {
        viewer.setTaskName(task)
    }

    function setBaseFrame(base_frame)
    {
        viewer.setBaseFrame(base_frame)
    }

    function resetCamera()
    {
        viewer.resetCamera()
    }

    Rectangle
    {
        id: scene
        anchors.fill: parent
        color: "white"
        border.color: "lightGrey"

        Scene3D
        {
            id: scene3d
            anchors.fill: parent
            focus: true
            aspects: ["input", "logic"]
            cameraAspectRatioMode: Scene3D.AutomaticAspectRatio

            Viewer3DEntity
            {
                id: viewer
            }
        }
    }



}
