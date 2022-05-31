#pragma once
#include <mcp_can.h>
#include <SPI.h>


class Wheel { 
  private:
    MCP_CAN CAN = MCP_CAN(53);
    int currentAngle;
    int targetAngle;
    unsigned char len = 0;
    unsigned char buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    
  public:
    void calibrate() {                                   // Запуск калибровки
      CAN.sendMsgBuf(4, 0, 0, buf);
      Serial.println("Calibrate");
    }

    void setPWM(int val) {                               // Установка напряжения
      buf[0] = val;
      CAN.sendMsgBuf(5, 0, 1, buf);
    }
   
    void setAngleNow(int angle) {                        // Установка угла
      targetAngle = angle;
      buf[0] = targetAngle;
      byte *tempBuf = (byte*)&angle;
      CAN.sendMsgBuf(3, 0, 2, tempBuf);
      Serial.print("Set angle: ");
      Serial.println(targetAngle);
    }

    int getAngleNow() {
      CAN.sendMsgBuf(1, 0, 0, buf);
      unsigned long time = millis();
      while (millis() - time < 50) {
        if (CAN_MSGAVAIL == CAN.checkReceive()) {       // Прорверка получения данных
          CAN.readMsgBuf(&len, buf);    // read data
          if (CAN.getCanId() == 0x02) {                 // Получение угла
            currentAngle = (buf[1] << 8) + buf[0];
            Serial.print("Obtained angle: ");
            Serial.println(currentAngle);
            return currentAngle;
          }
        }
      }
      Serial.println("CAN get angle timeout!!!");
      return 0;
    }

    void init() {
      while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
          Serial.println("CAN BUS Shield init fail");
          Serial.println("Init CAN BUS Shield again");
          delay(200);
      }
      Serial.println("CAN BUS Shield init ok!");
      targetAngle = getAngleNow();
    }

    void setAngle(int angle) {
      targetAngle = angle;
    }

    int getAngle(){
      return currentAngle;
    }

    void update() {
      getAngleNow();
      setAngleNow(targetAngle);
    } 
};
