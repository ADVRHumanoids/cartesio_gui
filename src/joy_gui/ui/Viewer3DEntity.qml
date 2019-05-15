import QtQuick 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import QtQuick.Scene3D 2.0
import Viewer3D.Utils3D 1.0
import QtQuick.Controls 2.5

Entity {

    id: sceneRoot

    property alias camera: camera

    property string link_name: ""
    property string ctrl_type: "global"
    property string mesh_url: ""

    Timer
    {
        id: timer
        interval: 33
        repeat: true
        running: true
        triggeredOnStart: true

        onTriggered:
        {
            viewer.setTransform(link_name, ctrl_type)
        }
    }

    function resetCamera()
    {
        camera.setFieldOfView(45)
        camera.setAspectRatio(16/9)
        camera.setNearPlane(0.1)
        camera.setFarPlane(1000.0)
        camera.setPosition(Qt.vector3d( 0.0, 0.0, -0.5 ))
        camera.setUpVector(Qt.vector3d( 0.0, 1.0, 0.0 ))
        camera.setViewCenter(Qt.vector3d( 0.0, 0.0, 0.0 ))
    }


    function setTaskName(task)
    {
        link_name = task
        viewer.setPathToStl(link_name)
    }

    function setBaseFrame(type)
    {
        ctrl_type = type
    }

    function setPathToStl(link_name)
    {
        mesh_url = utils.pathToStl(link_name)
        if(mesh_url === "")
            print("mesh url can not be found")
        else
            myMesh.setSource(mesh_url)
    }

    function setTransform(link_name, ctrl_type)
    {

        //1) Mesh is always global
        if(mesh_url !== "")
        {
            var T
            T = utils.getMeshGlobalPose(link_name)

            myTransform.setTranslation(utils.translation(utils.toCameraFrame(T)))
            myTransform.setRotation(utils.rotation(utils.toCameraFrame(T)))
            myTransform.setScale3D(utils.getScale(link_name))

            myMesh.setEnabled(true)

            var translation
            var rotation
            //2 Ref Frame could be global, local or base_link
            if(ctrl_type === "global")
            {
                translation = Qt.vector3d(0.05, 0, 0)
                rotation = xAxisTransform.fromAxisAndAngle(
                            Qt.vector3d(0, 0, -1), 90)
                xAxisTransform.setTranslation(translation)
                xAxisTransform.setRotation(rotation)

                translation = Qt.vector3d(0, 0, -0.05)
                rotation = yAxisTransform.fromAxisAndAngle(
                            Qt.vector3d(-1, 0, 0), 90)
                yAxisTransform.setTranslation(translation)
                yAxisTransform.setRotation(rotation)

                translation = Qt.vector3d(0, 0.05, 0)
                rotation = zAxisTransform.fromAxisAndAngle(
                            Qt.vector3d(0, 0, 0), 0)
                zAxisTransform.setTranslation(translation)
                zAxisTransform.setRotation(rotation)
            }
            else
            {
                // 1) Get pose of the object in global coordinates and put in camera frame
                if(ctrl_type === "local")
                {
                    T = utils.toCameraFrame(utils.getPose("world_odom", link_name))
                }
                if(ctrl_type === "base_link")
                {
                    T = utils.toCameraFrame(utils.getPose("base_link", link_name))
                }
                var Tr = utils.toMatrix(Qt.vector3d(0.0, 0.0, 0), utils.rotation(T))

                // 2) Rotate cylinder in the right axis (camera frame)
                translation = Qt.vector3d(0.05, 0.0, 0)
                rotation = xAxisTransform.fromAxisAndAngle(
                            Qt.vector3d(0, 0, 1), 90)
                var T1 = utils.toMatrix(translation, rotation)

                // 3) Put cylinder in global coordinates and premultiply by object pose in camera frame
                var TrT1 = utils.mul(Tr, utils.toGlobalFrame(T1))
                xAxisTransform.setMatrix(TrT1)

                // 4) Same procedure for y Axis
                translation = Qt.vector3d(0.0, 0.0, -0.05)
                rotation = yAxisTransform.fromAxisAndAngle(
                            Qt.vector3d(1, 0, 0), 90)
                T1 = utils.toMatrix(translation, rotation)

                TrT1 = utils.mul(Tr, utils.toGlobalFrame(T1))

                yAxisTransform.setMatrix(TrT1)


                // 5) Same procedure for z axis
                translation = Qt.vector3d(0.0, 0.05, 0.0)
                rotation = zAxisTransform.fromAxisAndAngle(
                            Qt.vector3d(0, 0, 0), 0)
                T1 = utils.toMatrix(translation, rotation)

                TrT1 = utils.mul(Tr, utils.toGlobalFrame(T1))

                zAxisTransform.setMatrix(TrT1)
            }
        }
    }

    Utils3D{
        id: utils
    }

    Camera {
        id: camera
        projectionType: CameraLens.PerspectiveProjection
        fieldOfView: 45
        aspectRatio: 16/9
        nearPlane : 0.1
        farPlane : 1000.0
        position: Qt.vector3d( 0.0, 0.0, -0.5 )
        upVector: Qt.vector3d( 0.0, 1.0, 0.0 )
        viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
    }

    OrbitCameraController {
        camera: camera
        linearSpeed: 0
        lookSpeed: 1e4
        zoomInLimit: -0.5
    }

    components: [
        RenderSettings {
            id: render
            renderPolicy: RenderSettings.Always
            activeFrameGraph: ForwardRenderer {
                clearColor: Qt.rgba(1, 1, 1, 0)
                camera: camera
            }
        },
        // Event Source will be set by the Qt3DQuickWindow
        InputSettings { }
    ]

    PhongAlphaMaterial {
        id: material
        ambient: Qt.rgba(0.5, 0.5, 0.5, 0)
        alpha: 0.8
    }

    Transform {
        id: myTransform
        scale: 0.001
        translation: utils.translation(utils.toCameraFrame(Qt.matrix4x4()))
        rotation: utils.rotation(utils.toCameraFrame(Qt.matrix4x4()))
    }

    Mesh {
        id: myMesh
        enabled: false
    }


    Entity {
        id: myEntity
        components: [ myMesh, material, myTransform ]
    }

    CylinderMesh {
        id: xAxis
        length: 0.1
        radius: 0.005
    }

    Transform {
        id: xAxisTransform
        scale: 1
        rotation: fromAxisAndAngle(Qt.vector3d(0, 0, 1), 90)
        translation: Qt.vector3d(0.05, 0, 0)
    }


    PhongMaterial {
        id: xAxisMaterial
        ambient: "#FF0000"
    }

    Entity {
        id: xAxisEntity
        components: [ xAxis, xAxisMaterial, xAxisTransform ]
    }

    CylinderMesh {
        id: yAxis
        length: 0.1
        radius: 0.005
    }

    Transform {
        id: yAxisTransform
        scale: 1
        rotation: fromAxisAndAngle(Qt.vector3d(1, 0, 0), 90)
        translation: Qt.vector3d(0, 0, -0.05)
    }

    PhongMaterial {
        id: yAxisMaterial
        ambient: "#00FF00"
    }

    Entity {
        id: yAxisEntity
        components: [ yAxis, yAxisMaterial, yAxisTransform ]
    }

    CylinderMesh {
        id: zAxis
        length: 0.1
        radius: 0.005
    }

    Transform {
        id: zAxisTransform
        scale: 1
        translation: Qt.vector3d(0, 0.05, 0)
    }

    PhongMaterial {
        id: zAxisMaterial
        ambient: "#0000FF"
    }

    Entity {
        id: zAxisEntity
        components: [ zAxis, zAxisMaterial, zAxisTransform ]
    }
}
