enum MotorState{
    shutdown,
    launching,
    work
};

class Engine {
public:
    // starter pins
    int p1, p2, p3;
//
//  shutdown               launching       work
//            --launch()-->
//                                    ----> 
//            <----------shutdown----------  
//
    MotorState state;
    unsigned long launch_time;

    Engine(int p1, int p2, int p3) :p1(p1), p2(p2), p3(p3) {
        pinMode(p1, OUTPUT);
        pinMode(p2, OUTPUT);
        pinMode(p3, OUTPUT);
    }

    void Update() {
        if (state == MotorState::shutdown){
            digitalWrite(p1, HIGH);
            digitalWrite(p2, HIGH);
            digitalWrite(p3, HIGH);
        }
        if (state == MotorState::launching){
            int cureent_time = millis();
            if (cureent_time - launch_time < 1000){
                digitalWrite(p1, LOW);
                digitalWrite(p2, LOW);
            }
            if (cureent_time - launch_time >= 1000){
                digitalWrite(p3, LOW);
            }
            if (cureent_time - launch_time >= 2000){
                digitalWrite(p1, HIGH);
                state = MotorState::work;
            }
        }
        if (state == MotorState::work){

        }
    }

    void launch() {
        launch_time = millis();
        state = MotorState::launching;
    }

    void shutdown() {
        state = MotorState::shutdown;
    }
};
