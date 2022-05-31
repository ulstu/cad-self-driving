#pragma once
#include "Motor.h"

class GearboxMotor {
public:
    enum Position
    {
    	undefined = -2,
        up = 1,
        middle = 0,
        down = -1,
    };
    
    enum State{
        moving = 1,
        fine_tuning = 2,
        idle = 0
    };

    Position current_position = -2;

    Position target_position = Position::middle;

    Position getpos()
    {
        if (digitalRead(lim_up) == HIGH) {
            return Position::up;
        }
        else if (digitalRead(lim_middle) == LOW) {
            return Position::middle;
        }
        else if (digitalRead(lim_down) == HIGH) {
            return Position::down;
        }
        return Position::undefined;
    }

    GearboxMotor(Motor motor, int l_up, int l_middle, int l_down):
        motor(motor),
        lim_up(l_up),
        lim_down(l_down),
        lim_middle(l_middle)
    {
        
        
    }

    void init() {
    	pinMode(lim_up, INPUT_PULLUP);
        pinMode(lim_down, INPUT_PULLUP);
        pinMode(lim_middle, INPUT_PULLUP);
        delay(10);
    	current_position = getpos();
        if (current_position == Position::undefined) {
        	motor.run(255, Motor::Direction::forward);
        	motor.update();
        	while (getpos() == Position::undefined) {
        	}
        	/*
        	if (getpos() == Position::up) {
        		while (getpos() != Position::middle) {
					motor.run(255, Motor::Direction::backward);
        		}
        	}
        	else if (getpos() == Position::down) {
        		while (getpos() != Position::middle) {
					motor.run(255, Motor::Direction::forward);
        		}
        	}
        	*/
        	motor.run(0, Motor::Direction::neutral);
        	delay(150);
        	motor.update();
        	current_position = getpos();
        }
        target_position = current_position;
        
    }

    Position get_position(){
        return current_position;
    }


    

    void update(){
    	/*
    	Serial.print(digitalRead(lim_up));
    	Serial.print(!digitalRead(lim_middle));
    	Serial.println(digitalRead(lim_down));
    	*/

        Position position = getpos();

        // ожидаем новое целевое положение мотора
        // как только целевое положение изменено
        // начинаем двигать мотор в нужную сторону

        if (state == State::idle) {
            if (current_position != target_position) {
                state = State::moving;
                if (current_position < target_position) {
                    motor.run(255, Motor::Direction::forward);
                }
                else{
                    motor.run(255, Motor::Direction::backward);
                }
            }
        }
        // мотор движется в сторону целевого положения
        // до тех пор, пока оно не будет достигнуто
        else if (state == State::moving) {
            if (position == target_position) {
                state = State::fine_tuning;
                position_switch_time = millis() + 150;
            }
        }
        // мотор движется еще некоторое время для доводки
        else if (state == State::fine_tuning) {
            if (position_switch_time < millis()) {
                state = State::idle;
                current_position = position;
                motor.run(0, Motor::Direction::neutral);
            }
        }
        motor.update();
    }

private:
    State state = State::idle;
    Motor motor;
    int lim_up, lim_down, lim_middle;
    unsigned long position_switch_time;
    int pos;
    
};
