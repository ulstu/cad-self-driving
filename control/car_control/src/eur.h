#pragma once

class Eur {
public:
    int rele;
    int value;
    int tahometr;
    int left_sensor, right_sensor;
    int lim_left, lim_right;
    int s1 = 170, s2 = 170;
    Eur(int rel, int tah, int sr, int sl, int lim_l, int lim_r) {
        rele = rel;
        tahometr = tah;
        left_sensor = sl;
        right_sensor = sr;
        lim_left = lim_l;
        lim_right = lim_r;
    }

    void init() {
        pinMode(lim_left, INPUT_PULLUP);
        pinMode(lim_right, INPUT_PULLUP);

        pinMode(rele, OUTPUT);
        pinMode(tahometr, OUTPUT);
        pinMode(left_sensor, OUTPUT);
        pinMode(right_sensor, OUTPUT);

        //tone(tahometr, 25);
        digitalWrite(rele, HIGH);
        analogWrite(left_sensor, s1);
        analogWrite(right_sensor, s2);
    }

    void turn(int val) {
        value = val;
        if ((value < 0 && digitalRead(lim_left)) || (value > 0 && digitalRead(lim_right))) {
            analogWrite(left_sensor, s1);
            analogWrite(right_sensor, s2);
        }
        else {
            analogWrite(left_sensor, s1 + val);
            analogWrite(right_sensor, s2 - val);
        }
    }

    void check_lim() {
        tone(tahometr, 25);
        if (value < 0 && digitalRead(lim_left) || value > 0 && digitalRead(lim_right)) {
            value = 0;
            analogWrite(left_sensor, s1);
            analogWrite(right_sensor, s2);
        }
    }

    void reboot() {
        digitalWrite(rele, LOW);
        delay(1000);
        tone(tahometr, 25);
        analogWrite(left_sensor, s1);
        analogWrite(right_sensor, s2);
        digitalWrite(rele, HIGH);
        delay(6000);
    }
};
