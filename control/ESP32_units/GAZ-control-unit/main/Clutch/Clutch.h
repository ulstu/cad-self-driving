#pragma once

class Clutch {
public:
    enum ClutchState {
        Unknown = 0,
        Press,
        Pause,
        Release
    };
    static void init();
    static void setState(ClutchState state);
    static void setParams(float disabledPos, float zeroPos, float fullEngPos, float minEngPos, float pressedPos);
private:
    static inline ClutchState state = Unknown;
    static void task(void* args);
    static inline float pauseValue;
    static inline float disabledPos;
    static inline float zeroPos;
    static inline float fullEngPos;
    static inline float minEngPos;
    static inline float pressedPos;
};