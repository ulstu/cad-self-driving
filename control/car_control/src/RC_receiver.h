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
		int state = digitalRead(2);
		if (state == HIGH) {
			times[0] = micros();
		}
		if (state == LOW) {
			gear_value = micros() - times[0];
		}
	}
	void pin_2_change() {
		int state = digitalRead(3);
		if (state == HIGH) {
			times[1] = micros();
		}
		if (state == LOW) {
			egn_value = micros() - times[1];
		}
	}
	void pin_3_change() {
		int state = digitalRead(18);
		if (state == HIGH) {
			times[2] = micros();
		}
		if (state == LOW) {
			unused_value = micros() - times[2];
		}
	}
	void pin_4_change() {
		int state = digitalRead(19);
		if (state == HIGH) {
			times[3] = micros();
		}
		if (state == LOW) {
			brake_value = micros() - times[3];
		}
	}
	void pin_5_change() {
		int state = digitalRead(20);
		if (state == HIGH) {
			times[4] = micros();
		}
		if (state == LOW) {
			clutch_value = micros() - times[4];
		}
	}
	void pin_6_change() {
		int state = digitalRead(21);
		if (state == HIGH) {
			times[5] = micros();
		}
		if (state == LOW) {
			eur_value = micros() - times[5];
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
		for (int i = 0; i < 6; i++) {
			if (micros() - times[i] > 100000) {
				return false;
			}
		}
		return true;
	}
}