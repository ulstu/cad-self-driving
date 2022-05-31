#pragma once

class PedalMotor {

public:

    PedalMotor(int pin, int pin_dir, int l_up, int l_down, int up_state, int down_state, bool inverse_direction):
        pin(pin), 
        pin_dir(pin_dir), 
        lim_up(l_up), 
        lim_down(l_down), 
        up_state(up_state), 
        down_state(down_state),
        inverse_direction(inverse_direction)
    {
        pinMode(l_up, INPUT_PULLUP);
        pinMode(l_down, INPUT_PULLUP);
    }

    void push(int speed = 100) {
        speed *= 2.5;
        this->speed = speed;
        direction = (inverse_direction ? 0 : 1);
    }

    void release(int speed = 100) {
        speed *= 2.5;
        this->speed = speed;
        direction = (inverse_direction ? 1 : 0);
    }

    void stop() {
        this->speed = 0;
    }

    int update() {
        GearboxMotor::Position current_position = get_position();
        //остановка движка в крайних положениях
        if (get_position() == GearboxMotor::Position::up && direction == (inverse_direction? 0 : 1)) {
            analogWrite(pin, 0);
        }
        else if (get_position() == GearboxMotor::Position::down && direction == (inverse_direction? 1 : 0)) {
            analogWrite(pin, 0);
        }
        else {
            analogWrite(pin, speed);
            digitalWrite(pin_dir, direction);
        }

        return current_confirmed_position;
    }

    GearboxMotor::Position get_position() {
        if (digitalRead(lim_up) == up_state) {
            return GearboxMotor::Position::up;
        }
        else if (digitalRead(lim_down) == down_state) {
            return GearboxMotor::Position::down;
        }
        else {
            return GearboxMotor::Position::middle;
        }
    }

private:
    int pin, pin_dir;
    int speed;
    int direction;
    int change_count;
    GearboxMotor::Position current_confirmed_position;
    int lim_up, lim_down;
    int up_state, down_state;
    bool inverse_direction;
};
