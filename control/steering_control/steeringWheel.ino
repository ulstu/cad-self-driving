#include <SPI.h>
#include "mcp_can.h"
#include <avr/eeprom.h>

const int SPI_CS_PIN = 5;

MCP_CAN CAN(SPI_CS_PIN);           // Set CS pin

unsigned char len = 0;             // Нужые переменные
unsigned char buf[8];
int currentAngle = 360 * 2.5;
int wheelOffset;
bool autoMode = false;

int getSteeringAngle() {                   // Получение угла поворота руля
  int resPWM = (analogRead(A0) - wheelOffset) * 2.3;
  return resPWM;
}

void beep(int _delay) {
  analogWrite(6, 127);
  delay(_delay);
  digitalWrite(6, 0);
  delay(150);
}

void setup() {
  
  beep(300);            // Сигнал запуска

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  TCCR1A = 0b00000001;            // Установка частоты для эмулятора датчика усилия 3900гц
  TCCR1B = 0b00000010;
  analogWrite(9, 127);            // Остановка усилителя
  analogWrite(10, 127);

  
  tone(3, 25);                    // Эмулятор тахометра 30гц
  //TCCR2B = 0b00000111;
  //TCCR2A = 0b00000001;
  //analogWrite(3, 157);

  wheelOffset = eeprom_read_word(0); // Считывание нулевого положения руля из eeporn

  
  Serial.begin(115200);
  
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(200);
  }
  Serial.println("CAN BUS Shield init ok!");
  
  CAN.init_Mask(0, 0, 0x3ff); // Фильтрция сообщений
  CAN.init_Mask(1, 0, 0x3ff);
  
  CAN.init_Filt(0, 0, 0x01);  // запрос на получение угла поворота
  CAN.init_Filt(1, 0, 0x03);  // запрос управление ЭУРом
  CAN.init_Filt(2, 0, 0x04);  // запрос на обновление крайнего положения руля
  CAN.init_Filt(3, 0, 0x05);  // принудительная установка напряжения ЭУРа
}

void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {     // Прорверка получения данных
    
    CAN.readMsgBuf(&len, buf);    // read data
    
    if (CAN.getCanId() == 0x01) {               // Зпарос на отправку текущего положения руля
      int angle = getSteeringAngle();
      byte *tempBuf = (byte*)&angle;
      CAN.sendMsgBuf(2, 0, 2, tempBuf);
    }
    
    if (CAN.getCanId() == 0x03) {               // Запрос на установку положения руля
      autoMode = true;
      currentAngle = (buf[1] << 8) + buf[0];
    }

    if (CAN.getCanId() == 0x04) {               // Запрос на калибровку положения руля
      analogWrite(9, 127);
      analogWrite(10, 127);

      beep(300);
      delay(15000);
      int leftBoundary = analogRead(A0);    // Считывание крайнего левого положения
      
      beep(300);
      beep(300);
      delay(15000);
      int rightBoundary = analogRead(A0);   // Считывание крайнего правого положения
      
      beep(300);
      beep(300);
      beep(300);

      int result = (leftBoundary + rightBoundary) / 2; // Простой и надежный алгоритм калибровки и определения нуля по двум крайним точкам (ПиНАКиОНпДКТ)
      
      eeprom_update_word(0, result);        // Запись среднего положения руля в eeporn
      wheelOffset = eeprom_read_word(0);
    }

    if (CAN.getCanId() == 0x05) {               // Ручная установка напряжения на усилителе
      autoMode = false;
      analogWrite(10, buf[0]);
      analogWrite(9, 255 - buf[0]);
      
      int difference = 127 + int(10.0 * sqrt(610 - abs(getSteeringAngle())) * (buf[0] - 127) / 127.0);
      Serial.println(difference);
      if (difference > 200)
        difference = 200;
      if (difference < 55)
        difference = 55;
      
      analogWrite(10, difference);
      analogWrite(9, 255 - difference);
    }
  }

  if (autoMode) {                               // Поддержание угла поворота руля
    int difference = 127 + int(10.0 * sqrt(abs(currentAngle - getSteeringAngle())));
    Serial.println(difference);
    if (difference > 200)
      difference = 200;

    if (currentAngle > getSteeringAngle()) {
      analogWrite(9, 255 - difference);
      analogWrite(10, difference);
    }
    else {
      analogWrite(9, difference);
      analogWrite(10, 255 - difference);
    }
  }
}
