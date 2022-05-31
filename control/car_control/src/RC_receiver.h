#pragma once

namespace RC_receiver {
	//используем глобальные переменные, тк это единственный способ вытащить данные из прерывания
	//код дублируется, тк нельзя внутрь функции прерывания передать инфу о том, куда именно класть данные
	//запись через классы, темплейты, лямбды не работает - проверено

    volatile unsigned long gear_value;   //переключатель 3 положения
    volatile unsigned long egn_value;    //переключатель 2 положения
    volatile unsigned long unused_value; //х левый
    volatile unsigned long brake_value;  //у левый
    volatile unsigned long clutch_value; //y правый
    volatile unsigned long eur_value;    //х правый

	volatile unsigned long times[6];
	
	void pin_1_change() {
		if (digitalRead(2) == LOW) {
			gear_value = micros() - times[0];
		}
	}
	void pin_2_change() {
		if (digitalRead(3) == LOW) {
			egn_value = micros() - times[0];
		}
	}
	void pin_3_change() {
		if (digitalRead(18) == LOW) {
			unused_value = micros() - times[0];
		}
	}
	void pin_4_change() {
		int state = digitalRead(19);
		if (state == HIGH) {
			times[0] = micros();
		}
		if (state == LOW) {
			brake_value = micros() - times[0];
		}
	}
	void pin_5_change() {
		if (digitalRead(20) == LOW) {
			clutch_value = micros() - times[0];
		}
	}
	void pin_6_change() {
		if (digitalRead(21) == LOW) {
			eur_value = micros() - times[0];
		}
	}

	void Setup(int* pins) {
		pinMode(pins[0], INPUT);
		attachInterrupt(digitalPinToInterrupt(pins[0]), pin_1_change, CHANGE);
		pinMode(pins[1], INPUT);
		attachInterrupt(digitalPinToInterrupt(pins[1]), pin_2_change, CHANGE);
		pinMode(pins[2], INPUT);
		attachInterrupt(digitalPinToInterrupt(pins[2]), pin_3_change, CHANGE);
		pinMode(pins[3], INPUT);
		attachInterrupt(digitalPinToInterrupt(pins[3]), pin_4_change, CHANGE);
		pinMode(pins[4], INPUT);
		attachInterrupt(digitalPinToInterrupt(pins[4]), pin_5_change, CHANGE);
		pinMode(pins[5], INPUT);
		attachInterrupt(digitalPinToInterrupt(pins[5]), pin_6_change, CHANGE);
	}

	bool Available() {
		if (micros() - times[0] > 300000) {
			return false;
		}
		return true;
	}
}