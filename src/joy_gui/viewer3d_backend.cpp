#include <viewer3d_backend.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/math/special_functions/sign.hpp>


Utils3D::Utils3D(QObject * parent):
    _n("cartesian"),
    _tf_prefix("ci/")
{
    T1.setToIdentity();
    T1.rotate(-90.0, QVector3D(1, 0, 0));

    std::string robot_urdf_string;
    _n.getParam("/robot_description", robot_urdf_string);
    _urdf.initString(robot_urdf_string);
}

QMatrix4x4 Utils3D::toCameraFrame(const QMatrix4x4& T)
{
    return T1*T;
}

QMatrix4x4 Utils3D::toGlobalFrame(const QMatrix4x4 &T)
{
    return T1.inverted()*T;
}

QVector3D Utils3D::translation(const QMatrix4x4 &T)
{
    return QVector3D(T(0,3), T(1,3), T(2,3));
}

QQuaternion Utils3D::rotation(const QMatrix4x4 &T)
{
    QMatrix3x3 R;
    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            R(i,j) = T(i,j);
    }
    return QQuaternion::fromRotationMatrix(R);
}

QString Utils3D::pathToStl(const QString &link_name)
{   
    boost::shared_ptr<const urdf::Link> link = _urdf.getLink(link_name.toStdString());
    boost::shared_ptr<const urdf::Link> controlled_link = link;

    if(!link)
    {
        ROS_ERROR("Link %s does not exists", link_name.toStdString().c_str());
        return "";
    }

    while(!link->visual)
    {
        if(!link->parent_joint)
        {
            ROS_ERROR("Unable to find a mesh for %s", link_name.toStdString().c_str());
            return "";
        }
        link = _urdf.getLink(link->parent_joint->parent_link_name);
    }

    QString stl_path;
    if(link->visual->geometry->type == urdf::Geometry::MESH)
    {
        boost::shared_ptr<urdf::Mesh> mesh =
                boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);



        stl_path = QString::fromStdString(getAbsolutePath(mesh->filename));
        std::cout<<"stl_path: "<<stl_path.toStdString()<<std::endl;
    }

    return "file://"+stl_path;
}

QMatrix4x4 Utils3D::getMeshGlobalPose(const QString &link_name)
{
    boost::shared_ptr<const urdf::Link> link = _urdf.getLink(link_name.toStdString());
    boost::shared_ptr<const urdf::Link> controlled_link = link;

    while(!link->visual)
    {
        if(!link->parent_joint)
        {
            ROS_ERROR("Unable to find a mesh for %s", link_name.toStdString().c_str());
            return QMatrix4x4();
        }
        link = _urdf.getLink(link->parent_joint->parent_link_name);
    }


    QMatrix4x4 T = getPose(controlled_link->name, link->name);
    QMatrix4x4 T_marker = URDFPoseToQMatrix4x4(link->visual->origin);

    T = T*T_marker;

    QMatrix4x4 TG = getPose("world_odom", link->name);
    TG(0,3) = 0;
    TG(1,3) = 0;
    TG(2,3) = 0;

    TG = TG*T;

    return TG;
}

QVector3D Utils3D::getScale(const QString &link_name)
{
    boost::shared_ptr<const urdf::Link> link = _urdf.getLink(link_name.toStdString());
    boost::shared_ptr<const urdf::Link> controlled_link = link;

    while(!link->visual)
    {
        if(!link->parent_joint)
        {
            ROS_ERROR("Unable to find a mesh for %s", link_name.toStdString().c_str());
            return QVector3D(0,0,0);
        }
        link = _urdf.getLink(link->parent_joint->parent_link_name);
    }

    QString stl_path;
    if(link->visual->geometry->type == urdf::Geometry::MESH)
    {
        boost::shared_ptr<urdf::Mesh> mesh =
                boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

        return QVector3D(mesh->scale.x, mesh->scale.y, mesh->scale.z);

    }

    return QVector3D(1,1,1);
}

QVector3D Utils3D::getNormalizedScale(const QString &link_name)
{
    QVector3D v = getScale(link_name);
    v.setX(boost::math::sign(v.x()));
    v.setY(boost::math::sign(v.y()));
    v.setZ(boost::math::sign(v.z()));
    return v;
}

QMatrix4x4 Utils3D::getPose(const std::string &base_link, const std::string &distal_link)
{
    for(unsigned int i = 0; i < 10; ++i)
    {
        try{
            ros::Time now = ros::Time::now();
            _listener.waitForTransform(_tf_prefix+base_link,
                                       _tf_prefix+distal_link,
                                       ros::Time(0),ros::Duration(1.0));

            _listener.lookupTransform(_tf_prefix+base_link,
                                      _tf_prefix+distal_link,
                                      ros::Time(0), _transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    QMatrix4x4 transform_QT = fromTransformToQMatrix4x4(_transform);

    return transform_QT;
}

QMatrix4x4 Utils3D::fromTransformToQMatrix4x4(const tf::Transform &T)
{
    QMatrix4x4 QT;

    QT(0,3) = T.getOrigin().x();
    QT(1,3) = T.getOrigin().y();
    QT(2,3) = T.getOrigin().z();

    QQuaternion quat(T.getRotation().getW(),
                     T.getRotation().getX(),
                     T.getRotation().getY(),
                     T.getRotation().getZ());

    QMatrix3x3 R = quat.toRotationMatrix();

    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            QT(i,j) = R(i,j);
    }

    return QT;
}

QMatrix4x4 Utils3D::URDFPoseToQMatrix4x4(const urdf::Pose &Pose)
{
    QMatrix4x4 QT;

    QT(0,3) = Pose.position.x;
    QT(1,3) = Pose.position.y;
    QT(2,3) = Pose.position.z;

    QQuaternion quat(Pose.rotation.w,
                     Pose.rotation.x,
                     Pose.rotation.y,
                     Pose.rotation.z);

    QMatrix3x3 R = quat.toRotationMatrix();

    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
            QT(i,j) = R(i,j);
    }

    return QT;

}

std::string Utils3D::getAbsolutePath(std::string filename)
{
    filename.erase(0, 10); // remove 'package://'
    std::vector<std::string> tokens;

    // tokenize path w.r.t. '/'
    boost::split(tokens, filename, boost::is_any_of("/"));

    // first token is package name
    auto pkg_name = tokens[0];
    std::cout << "Package name:" << pkg_name << std::endl;

    // find pkg path
    auto abs_path = ros::package::getPath(pkg_name);
    std::cout << "Package path:" << abs_path << std::endl;

    // append the rest of the path
    for(int i = 1; i < tokens.size(); i++)
    {
        abs_path += "/";
        abs_path += tokens[i];
        std::cout << "Adding token: '/" << tokens[i] << "'" << std::endl;
    }

    std::cout << "Package path:" << abs_path << std::endl;

    return abs_path;

}
