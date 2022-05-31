#include "src/eur2.h"
#include "src/gearbox.h"
#include "src/Engine.h"
#include "src/GearboxMotor.h"
#include "src/PedalMotor.h"
#include "src/RC_receiver.h"

Motor motor_x(14, 15);
Motor motor_y(16, 17);
GearboxMotor motor_gearbox_x(motor_x, 34, 32, 33);
GearboxMotor motor_gearbox_y(motor_y, 36, 31, 35);

Gearbox gearbox = Gearbox(motor_gearbox_x, motor_gearbox_y);
int gear_num = 0;

Wheel wheel;
//Eur eur(27, 6, 8, 11, 49, 48);

Engine engine(26, 25, 24);

//48 - 53 концевики
//пин концевики(2) скорость нажать отжать
PedalMotor clutch_motor(46, 47, 50, 51, HIGH, HIGH, false);
PedalMotor brake_motor(44, 45, 53, 52, LOW, HIGH, false);

void setup() {
    pinMode(43, INPUT_PULLUP);
    Serial.begin(115200);

    wheel.init(); // Инициализация руля
    
    int pins[] = { 2, 3, 18, 19, 20, 21 };
    RC_receiver::Setup(pins);

    pinMode(23, OUTPUT);
    digitalWrite(23, HIGH);

    for (int i = 0; i < 6; i++) {
        pinMode(31 + i, INPUT_PULLUP);
    }
}

int state = 0;
void loop() {
    if (RC_receiver::Available() && digitalRead(43) != HIGH) { //кнопка нажата
        Serial.println("work");

        if (RC_receiver::clutch_value < 1250) {
            clutch_motor.push(50);
        }
        else if (RC_receiver::clutch_value < 1750) {
            clutch_motor.stop();
        }
        else {
            clutch_motor.release(17);
        }

        if (RC_receiver::brake_value < 1250) {
            brake_motor.push(50);
        }
        else if (RC_receiver::brake_value < 1750) {
            brake_motor.stop();
        }
        else {
            brake_motor.release(25);
        }

        if (RC_receiver::egn_value < 1500) {
            if (state == 1) {
                Serial.println("Shutdown engine");
                state = 0;
                engine.shutdown();
            }
        }
        else {
            if (state == 0) {
                Serial.println("launch engine");
                //eur.reboot();
                engine.launch();
                state = 1;
            }
        }

        wheel.setPWM(int(((RC_receiver::eur_value - 994) / 1000.0) * 195 + 30));

        if (RC_receiver::gear_value > 1750) {
            gear_num = 1;
        }
        else if (RC_receiver::gear_value > 1250) {
            gear_num = 0;
        }
        else {
            gear_num = 6;
        }
    }
    else {
        Serial.println("STOP");
        gear_num = 0;
        engine.shutdown();
        brake_motor.push(100);
        clutch_motor.push(100);
    }
    //eur.check_lim();
    clutch_motor.update();
    brake_motor.update();
    gearbox.set_gear(gear_num);
}
