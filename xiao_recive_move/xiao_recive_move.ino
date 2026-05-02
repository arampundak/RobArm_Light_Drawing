// created 20260502
// works well with wave_hello.py in 20260502 folder. motor 1 is shaking but we took care of it
// What this does
// waits for a line from Python
// if it gets MOVE,id,pos          -> moves that servo at default speed (300)
// if it gets MOVE,id,pos,speed    -> moves that servo at the given speed
// if it gets TORQUE,id,enable     -> 1 = hold position, 0 = release (free-spin)
// speed is in SCServo units (0..4000). 0 = max speed (jerky); ~300 = smooth.
// prints confirmation back


#include <SCServo.h>

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32S3)
#define COMSerial Serial0
#else
#define COMSerial Serial1
#endif

#define S_RXD D7
#define S_TXD D6

SMS_STS st;

String incomingLine = "";

void setup() {
  Serial.begin(115200);
  delay(2000);

  COMSerial.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &COMSerial;

  Serial.println("READY");
}

void handleMoveCommand(String line) {
  int firstComma  = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1) {
    Serial.println("ERROR,BAD_FORMAT");
    return;
  }

  // Optional fourth field (speed). thirdComma == -1 means it was omitted.
  int thirdComma = line.indexOf(',', secondComma + 1);

  String command = line.substring(0, firstComma);
  String idText  = line.substring(firstComma + 1, secondComma);
  String posText;
  String speedText;

  if (thirdComma == -1) {
    posText   = line.substring(secondComma + 1);
    speedText = "";
  } else {
    posText   = line.substring(secondComma + 1, thirdComma);
    speedText = line.substring(thirdComma + 1);
  }

  command.trim();
  idText.trim();
  posText.trim();
  speedText.trim();

  if (command != "MOVE") {
    Serial.println("ERROR,UNKNOWN_COMMAND");
    return;
  }

  int motorId   = idText.toInt();
  int targetPos = posText.toInt();
  int speed     = (speedText.length() == 0) ? 300 : speedText.toInt();

  // Clamp speed to the SCServo range (0..4000). 0 = max speed.
  if (speed < 0)    speed = 0;
  if (speed > 4000) speed = 4000;

  st.WritePosEx(motorId, targetPos, speed, 50);

  Serial.print("OK,");
  Serial.print(motorId);
  Serial.print(",");
  Serial.print(targetPos);
  Serial.print(",");
  Serial.println(speed);
}

void handleTorqueCommand(String line) {
  int firstComma  = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1) {
    Serial.println("ERROR,BAD_FORMAT");
    return;
  }

  String idText     = line.substring(firstComma + 1, secondComma);
  String enableText = line.substring(secondComma + 1);
  idText.trim();
  enableText.trim();

  int motorId = idText.toInt();
  int enable  = enableText.toInt();

  // 1 = torque on (servo holds position), 0 = torque off (joint spins freely).
  st.EnableTorque(motorId, enable ? 1 : 0);

  Serial.print("OK,TORQUE,");
  Serial.print(motorId);
  Serial.print(",");
  Serial.println(enable);
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      incomingLine.trim();

      if (incomingLine.length() > 0) {
        if (incomingLine.startsWith("MOVE,")) {
          handleMoveCommand(incomingLine);
        } else if (incomingLine.startsWith("TORQUE,")) {
          handleTorqueCommand(incomingLine);
        } else {
          Serial.println("ERROR,UNKNOWN_COMMAND");
        }
      }

      incomingLine = "";
    } else {
      incomingLine += c;
    }
  }
}