/*
    FILE: utils.h
    --------------------------
    function utils for detectors
*/

#ifndef ONBOARD_DETECTOR_UTILS_H
#define ONBOARD_DETECTOR_UTILS_H
#include <iomanip>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/Eigen>

namespace onboardDetector {
    const double PI_const = 3.1415926;

    struct box3D {
        /* data */
        float x = 0, y = 0, z = 0;
        float x_width = 0, y_width = 0, z_width = 0;
        float id = 0;
        float Vx = 0, Vy = 0;
        float Ax = 0, Ay = 0;
        bool is_human = false; // false: not detected by yolo as dynamic, true: detected by yolo
        bool is_dynamic = false; // false: not detected as dynamic(either yolo or classificationCB), true: detected as dynamic
        bool fix_size = false; // flag to force future boxes to fix size
        bool is_dynamic_candidate = false;
    };

    inline geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        if (yaw > PI_const) {
            yaw = yaw - 2 * PI_const;
        }
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    inline double rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quat) {
        // return is [0, 2pi]
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    inline void rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quat, double &roll, double &pitch, double &yaw) {
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    inline double angleBetweenVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return std::atan2(a.cross(b).norm(), a.dot(b));
    }
}

#endif
