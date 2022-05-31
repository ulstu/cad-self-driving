#pragma once

class Motor{
public:
    enum Direction{
        forward, backward, neutral
    };

    Motor(int speed_pin, int dir_pin, bool inversed = false):
        speed_pin(speed_pin),
        dir_pin(dir_pin),
        inversed(inversed)
    {
        pinMode(speed_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        digitalWrite(dir_pin, LOW);
        analogWrite(speed_pin, 0);
    }

    void run(int speed, Direction direction){
        if (direction != Direction::neutral){
            this->speed = constrain(speed, 0, 100);
        }else{
            this->speed = 0;
        }
        this->direction = direction;
    }

    Direction get_direction(){
        return direction;
    }

    void Update(){
        if (direction == Direction::forward){
            digitalWrite(dir_pin, inversed ? HIGH : LOW);
            analogWrite(speed_pin, speed);
        }
        if (direction == Direction::backward){
            digitalWrite(dir_pin, inversed ? LOW : HIGH);
            analogWrite(speed_pin, speed);
        }
        if (direction == Direction::neutral){
            analogWrite(speed_pin, 0);
        }
    }
private:

    int speed_pin;
    int dir_pin;
    int speed;
    Direction direction;
    bool inversed;
};
