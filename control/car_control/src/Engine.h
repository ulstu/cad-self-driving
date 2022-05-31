enum MotorState{
    shutdown = 0,
    launching = 1,
    work = 2
};

class Engine {
private:
    // starter pins
    int p1, p2, p3;
  
    MotorState state = MotorState::shutdown;
    unsigned long start_time = 0;;

public:
    Engine(int p1, int p2, int p3) :p1(p1), p2(p2), p3(p3) {
        pinMode(p1, OUTPUT);
        digitalWrite(p1, HIGH);
        pinMode(p2, OUTPUT);
        digitalWrite(p2, HIGH);
        pinMode(p3, OUTPUT);
        digitalWrite(p3, HIGH);
    }

    void update() {
        if (state == MotorState::shutdown) {
            digitalWrite(p1, HIGH);
            digitalWrite(p2, HIGH);
            digitalWrite(p3, HIGH);
        }
        if (state == MotorState::launching) {
            unsigned long cureent_time = millis();
            if (cureent_time - start_time < 1000) {
                digitalWrite(p3, LOW);
                digitalWrite(p2, LOW);
            }
            else if (cureent_time - start_time >= 2000) {
                digitalWrite(p1, HIGH);
                state = MotorState::work;
            }
            else if (cureent_time - start_time >= 1000) {
                digitalWrite(p1, LOW);
            }
        }
    }

    void start() {
        start_time = millis();
        state = MotorState::launching;
    }

    void stop() {
        state = MotorState::shutdown;
    }
};
