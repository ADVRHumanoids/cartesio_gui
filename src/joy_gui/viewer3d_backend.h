#ifndef _3D_view_UTILS_H_
#define _3D_view_UTILS_H_

#include <QObject>
#include <QVector>
#include <QMatrix4x4>
#include <urdf/model.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <QUrl>


class Utils3D: public QObject
{
    Q_OBJECT
public:

    explicit Utils3D(QObject * parent = nullptr);

    Q_INVOKABLE QMatrix4x4 toCameraFrame(const QMatrix4x4& T);
    Q_INVOKABLE QMatrix4x4 toGlobalFrame(const QMatrix4x4& T);

    Q_INVOKABLE QVector3D translation(const QMatrix4x4& T);
    Q_INVOKABLE QQuaternion rotation(const QMatrix4x4& T);

    Q_INVOKABLE QString pathToStl(const QString& link_name);
    Q_INVOKABLE QMatrix4x4 getMeshGlobalPose(const QString& link_name);

    Q_INVOKABLE QVector3D getScale(const QString& link_name);
    Q_INVOKABLE QVector3D getNormalizedScale(const QString& link_name);

    Q_INVOKABLE QMatrix4x4 getPose(const QString& base_link, const QString& distal_link)
    {
        return getPose(base_link.toStdString(), distal_link.toStdString());
    }

    Q_INVOKABLE QMatrix4x4 toMatrix(const QVector3D& translation, const QQuaternion& rotation)
    {
        QMatrix4x4 T;
        T.rotate(rotation);

        T(0,3) = translation.x();
        T(1,3) = translation.y();
        T(2,3) = translation.z();

        return T;
    }

    Q_INVOKABLE QMatrix4x4 mul(const QMatrix4x4& A, const QMatrix4x4& B)
    {
        return A*B;
    }


private:
    /**
     * @brief T1 transform from world to camera frame
     */
    QMatrix4x4 T1;

    ros::NodeHandle _n;
    urdf::Model _urdf;
    tf::TransformListener _listener;
    tf::StampedTransform _transform;
    std::string _tf_prefix;

    QMatrix4x4 fromTransformToQMatrix4x4(const tf::Transform& T);
    QMatrix4x4 URDFPoseToQMatrix4x4(const urdf::Pose& Pose);

    std::string getAbsolutePath(std::string filename);
    QMatrix4x4 getPose(const std::string& base_link, const std::string& distal_link);

};

#endif
