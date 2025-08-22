/*
  Sunray Robot
  Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
  Licensed GPLv3 for open source use
  or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
*/

#include "QuaternionMath.h"
#include <math.h>

void QuaternionMath::toEulerAngles(float w, float x, float y, float z, float& roll, float& pitch, float& yaw) {
    double ysqr = y * y;

    // roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);
    yaw = atan2(t3, t4);
}

float QuaternionMath::magnitude(float w, float x, float y, float z) {
    return sqrt(w * w + x * x + y * y + z * z);
}

void QuaternionMath::normalize(float& w, float& x, float& y, float& z) {
    float mag = magnitude(w, x, y, z);
    if (mag > 0.0f) {
        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;
    }
}

void QuaternionMath::conjugate(float w, float x, float y, float z, float& conjW, float& conjX, float& conjY, float& conjZ) {
    conjW = w;
    conjX = -x;
    conjY = -y;
    conjZ = -z;
}

void QuaternionMath::multiply(float w1, float x1, float y1, float z1, 
                            float w2, float x2, float y2, float z2,
                            float& resultW, float& resultX, float& resultY, float& resultZ) {
    resultW = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    resultX = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    resultY = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    resultZ = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

void QuaternionMath::rotateVector(float qw, float qx, float qy, float qz, float& vx, float& vy, float& vz) {
    // Quaternion-Vektor-Rotation: v' = q * v * q*
    // Optimierte Version ohne explizite Quaternion-Multiplikation
    
    // Berechne 2 * (q.xyz cross v)
    float cx = 2.0f * (qy * vz - qz * vy);
    float cy = 2.0f * (qz * vx - qx * vz);
    float cz = 2.0f * (qx * vy - qy * vx);
    
    // Berechne 2 * (q.xyz dot v)
    float dot = 2.0f * (qx * vx + qy * vy + qz * vz);
    
    // v' = v + qw * (q.xyz cross v) + q.xyz cross (q.xyz cross v)
    vx = vx + qw * cx + qy * cz - qz * cy;
    vy = vy + qw * cy + qz * cx - qx * cz;
    vz = vz + qw * cz + qx * cy - qy * cx;
}