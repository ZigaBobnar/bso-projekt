#pragma once

#include "common.hpp"
#include "device/utils/xyzfloat.hpp"
// #include "math.h"
#include <cmath>

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

struct Quaternion {
    /* Real part */
    float w;

    /* Imaginary parts */
    float x;
    float y;
    float z;
};

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

inline Quaternion to_quaternion(float roll, float pitch, float yaw) {
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

inline Quaternion to_quaternion(EulerAngles value) {
    return to_quaternion(value.roll, value.pitch, value.yaw);
}

inline EulerAngles to_euler_angles(Quaternion value) {
    EulerAngles angles;
    
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (value.w * value.x + value.y * value.z);
    double cosr_cosp = 1 - 2 * (value.x * value.x + value.y * value.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (value.w * value.y - value.x * value.z));
    double cosp = std::sqrt(1 - 2 * (value.w * value.y - value.x * value.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (value.w * value.z + value.x * value.y);
    double cosy_cosp = 1 - 2 * (value.y * value.y + value.z * value.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);


    // float sinr_cosp = 2 * (value.w * value.x + value.y * value.z);
    // float cosr_cosp = 1 - 2 * (value.x * value.x + value.y * value.y);
    // angles.roll = atan2(sinr_cosp, cosr_cosp);

    // float sinp = 2 * (value.w * value.y - value.z * value.x);
    // if (fabs(sinp) >= 1)
    //     angles.pitch = copysign(M_PI / 2, sinp);
    // else
    //     angles.pitch = asin(sinp);

    // float siny_cosp = 2 * (value.w * value.z + value.x * value.y);
    // float cosy_cosp = 1 - 2 * (value.y * value.y + value.z * value.z);
    // angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}
