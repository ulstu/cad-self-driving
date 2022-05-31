#pragma once

class PedalMotor {

private:
    int pwmPin, dirPin, analogPin, limitSwitch;
    int min, max;
    int targetPosition;
    bool inversed;
    bool fast = false;

public:

    PedalMotor(int pwmPin, int min, int max, int dirPin, int analogPin, int limitSwitch, bool inversed) {
        this->pwmPin      = pwmPin;
        this->dirPin      = dirPin;
        this->analogPin   = analogPin;
        this->min         = min;
        this->max         = max;
        this->limitSwitch = limitSwitch;
        this->inversed    = inversed;
        targetPosition    = 0;
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        if (limitSwitch != NULL) {
        	pinMode(limitSwitch, INPUT_PULLUP);
        }
    }

    void set(int position) {
    	if (position == -1) {
    		fast = true;
    	} else {
    		fast = false;
    	}
    	targetPosition = abs(position) * (max / 255.0);
    	if (targetPosition > max) {
    		targetPosition = max;
    	}
    	else if (targetPosition < min) {
    		targetPosition = min;
    	}
    }

    void update() {
        int currentPosition = analogRead(analogPin);
        int speed = 4 * abs(currentPosition - targetPosition);
        if (speed > 255) {
        	speed = 255;
        }
        //else if (speed < 50) {
        //    speed = 0;
        //}
        if(currentPosition > targetPosition) {
    		digitalWrite(dirPin, inversed? 0 : 1);
        }
        else if (currentPosition < targetPosition) {
        	digitalWrite(dirPin, inversed? 1 : 0);
        }

        if (limitSwitch != NULL) {
        	if (!digitalRead(limitSwitch) && currentPosition < targetPosition) {
        		speed = 0;
        	}
        }
        else if ((currentPosition > targetPosition) && (currentPosition < (0.92 * max)) && (currentPosition > (0.01 * max)) && fast == false) {
        	speed *= 0.06;
        }
        analogWrite(pwmPin, speed);
    }

    int getPosition() {
        int result = analogRead(analogPin) * (255.0 / max);
        if (result > max) {
            result = max;
        }
        else if (result < min) {
            result = min;
        }
    	return result;
    }

    /*
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
*/
};
