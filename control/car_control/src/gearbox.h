#pragma once
#include "GearboxMotor.h"

class Gearbox {
public:

    //  -1 0 1 x
    //-1 1 3 5
    // 0 0-0-0
    // 1 2 4 6
    // y
    GearboxMotor::Position target_x, target_y;

    enum State {
        ready = 0, //ожидание (коробка находится в одной из скоростей и не переключает скорость)
        run_x = 1, //движение x
        run_y = 2 //движение y
    };

    State state = State::ready;
    GearboxMotor gmotor_x, gmotor_y;
    Gearbox(GearboxMotor x, GearboxMotor y) :gmotor_x(x), gmotor_y(y) {
        target_x = GearboxMotor::Position::middle;
        target_y = GearboxMotor::Position::middle;
    }

    void set_gear(int target) {
        if (state == State::ready) {
            if (target == 0) {
                target_y = GearboxMotor::Position::middle;
                target_x = gmotor_x.get_position();
            }
            else if (target == 1) {
                target_x = GearboxMotor::Position::down;
                target_y = GearboxMotor::Position::down;
            }
            else if (target == 2) {
                target_x = GearboxMotor::Position::down;
                target_y = GearboxMotor::Position::up;
            }
            else if (target == 3) {
                target_x = GearboxMotor::Position::middle;
                target_y = GearboxMotor::Position::down;
            }
            else if (target == 4) {
                target_x = GearboxMotor::Position::middle;
                target_y = GearboxMotor::Position::up;
            }
            else if (target == 5) {
                target_x = GearboxMotor::Position::up;
                target_y = GearboxMotor::Position::down;
            }
            else if (target == 6) {
                target_x = GearboxMotor::Position::up;
                target_y = GearboxMotor::Position::up;
            }
        }
    }

    void init() {
        gmotor_y.init();
        gmotor_x.init();
    }

    void update() {
        if (state == State::ready) {
            //не совпадает х, у не в нейтралке
            //переводим на нейтралку
            
            if (gmotor_x.get_position() != target_x && gmotor_y.get_position() != GearboxMotor::Position::middle) {
                gmotor_y.target_position = GearboxMotor::Position::middle;
            }
            //не совпадает х, у в нейтралке
            //переключаем по х
            else if (gmotor_x.get_position() != target_x && gmotor_y.get_position() == GearboxMotor::Position::middle) {
                gmotor_x.target_position = target_x;
                state = State::run_x;
            }
            //совпадает х, не совпадает у
            //переключаем по у
            else if (gmotor_x.get_position() == target_x && gmotor_y.get_position() != target_y) {
                gmotor_y.target_position = target_y;
                state = State::run_y;
            }
        }

        //совпадают обе оси
        if (state == State::run_y && gmotor_y.get_position() == gmotor_y.target_position) {
        	state = State::ready;
        }
        else if (state == State::run_x && gmotor_x.get_position() == gmotor_x.target_position) {
        	state = State::ready;
        }
        
        gmotor_y.update();
        gmotor_x.update();
        
    }
};
