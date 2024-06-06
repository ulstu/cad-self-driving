#pragma once


class Brake {
public:
    static void setBrakeTorque(float brakeTorque);
    static void setParams(float maxCurrent, float zeroPosition);
private:
    static inline float maxCurrent = 1;
    static inline float zeroPosition = 0.1;
    static inline float lastBrakeTorque = -1;
};