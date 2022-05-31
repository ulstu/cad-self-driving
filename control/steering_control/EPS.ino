#define TAH 5

#define FORCE_LEFT 10
#define FORCE_RIGHT 9

int turnVal = 0;


void setup() {
    Serial.begin(9600);
    pinMode(TAH, OUTPUT);
    pinMode(FORCE_LEFT, OUTPUT);
    pinMode(FORCE_RIGHT, OUTPUT);

}

void loop() {
    if (Serial.available()) {
        turnVal = Serial.parseInt();
        Serial.readString();
        
    }

    tone(TAH, 25);

    int a = 160;
    analogWrite(FORCE_LEFT, a + turnVal);//172 + turnVal);
    analogWrite(FORCE_RIGHT, a - turnVal); //133 - turnVal);
}
