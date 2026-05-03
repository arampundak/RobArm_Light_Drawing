// created 20260502
// works well with wave_hello.py in 20260502 folder. took care of shaking
// What this does
// waits for a line from Python
// if it gets MOVE,id,pos              -> default speed (2500), default acc (500)
// if it gets MOVE,id,pos,speed        -> given speed,         default acc (500)
// if it gets MOVE,id,pos,speed,acc    -> given speed and acc
// if it gets TORQUE,id,enable         -> 1 = hold position, 0 = release (free-spin)
// if it gets READ,id                  -> replies OK,READ,id,pos,load,volt_dV,temp_C
//                                        any field can be -1 if the servo did not respond
// if it gets LIMITS,id                -> replies OK,LIMITS,id,min_angle,max_angle
//                                        the EEPROM angle limits — servo silently
//                                        clamps WritePosEx targets to this range
// speed is reversed on these servos: 0 = fastest/jerkiest, 4000 = slowest.
// These values worked well in testing: st.WritePosEx(id, pos, 2500, 500).
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

#define DEFAULT_SPEED 2500
#define DEFAULT_ACC 500

void setup()
{
  Serial.begin(115200);
  delay(2000);

  COMSerial.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &COMSerial;

  Serial.println("READY");


}

void loop()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n')
    {
      incomingLine.trim();

      if (incomingLine.length() > 0)
      {
        if (incomingLine.startsWith("MOVE,"))
        {
          handleMoveCommand(incomingLine);
        }
        else if (incomingLine.startsWith("TORQUE,"))
        {
          handleTorqueCommand(incomingLine);
        }
        else if (incomingLine.startsWith("READ,"))
        {
          handleReadCommand(incomingLine);
        }
        else if (incomingLine.startsWith("LIMITS,"))
        {
          handleLimitsCommand(incomingLine);
        }
        else
        {
          Serial.println("ERROR,UNKNOWN_COMMAND");
        }
      }

      incomingLine = "";
    }
    else
    {
      incomingLine += c;
    }
  }
}

void handleMoveCommand(String line)
{
  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1)
  {
    Serial.println("ERROR,BAD_FORMAT");
    return;
  }

  // Optional fourth field (speed). thirdComma == -1 means speed was omitted.
  int thirdComma = line.indexOf(',', secondComma + 1);
  // Optional fifth field (acc).   fourthComma == -1 means acc was omitted.
  int fourthComma = (thirdComma == -1) ? -1 : line.indexOf(',', thirdComma + 1);

  String command = line.substring(0, firstComma);
  String idText = line.substring(firstComma + 1, secondComma);
  String posText;
  String speedText;
  String accText;

  if (thirdComma == -1)
  {
    posText = line.substring(secondComma + 1);
    speedText = "";
    accText = "";
  }
  else if (fourthComma == -1)
  {
    posText = line.substring(secondComma + 1, thirdComma);
    speedText = line.substring(thirdComma + 1);
    accText = "";
  }
  else
  {
    posText = line.substring(secondComma + 1, thirdComma);
    speedText = line.substring(thirdComma + 1, fourthComma);
    accText = line.substring(fourthComma + 1);
  }

  command.trim();
  idText.trim();
  posText.trim();
  speedText.trim();
  accText.trim();

  if (command != "MOVE")
  {
    Serial.println("ERROR,UNKNOWN_COMMAND");
    return;
  }

  int motorId = idText.toInt();
  int targetPos = posText.toInt();
  int speed = (speedText.length() == 0) ? DEFAULT_SPEED : speedText.toInt();
  int acc = (accText.length() == 0) ? DEFAULT_ACC : accText.toInt();

  // Clamp speed to the SCServo range (0..4000). 0 is fastest, 4000 is slowest.
  if (speed < 0)
    speed = 0;
  if (speed > 4000)
    speed = 4000;

  // Do not cap acc at 255 here: direct st.WritePosEx(..., 500) was tested
  // and works best for this arm, so preserve the caller's value.
  if (acc < 0)
    acc = 0;

  st.WritePosEx(motorId, targetPos, speed, acc);

  Serial.print("OK,");
  Serial.print(motorId);
  Serial.print(",");
  Serial.print(targetPos);
  Serial.print(",");
  Serial.print(speed);
  Serial.print(",");
  Serial.println(acc);
}

void handleTorqueCommand(String line)
{
  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1)
  {
    Serial.println("ERROR,BAD_FORMAT");
    return;
  }

  String idText = line.substring(firstComma + 1, secondComma);
  String enableText = line.substring(secondComma + 1);
  idText.trim();
  enableText.trim();

  int motorId = idText.toInt();
  int enable = enableText.toInt();

  // 1 = torque on (servo holds position), 0 = release.
  st.EnableTorque(motorId, enable ? 1 : 0);

  Serial.print("OK,TORQUE,");
  Serial.print(motorId);
  Serial.print(",");
  Serial.println(enable);
}

void handleReadCommand(String line)
{
  int firstComma = line.indexOf(',');
  if (firstComma == -1)
  {
    Serial.println("ERROR,BAD_FORMAT");
    return;
  }

  String idText = line.substring(firstComma + 1);
  idText.trim();
  int motorId = idText.toInt();

  // SCServo SDK returns -1 on bus failure for any of these. Caller treats
  // pos == -1 as "this motor did not answer."
  int pos  = st.ReadPos(motorId);
  int load = st.ReadLoad(motorId);
  int volt = st.ReadVoltage(motorId);
  int temp = st.ReadTemper(motorId);

  Serial.print("OK,READ,");
  Serial.print(motorId);
  Serial.print(",");
  Serial.print(pos);
  Serial.print(",");
  Serial.print(load);
  Serial.print(",");
  Serial.print(volt);
  Serial.print(",");
  Serial.println(temp);
}

void handleLimitsCommand(String line)
{
  // LIMITS,id  -> reads the SMS_STS angle-limit registers from EEPROM:
  //   0x09/0x0A = MIN_ANGLE_LIMIT (16-bit, little endian)
  //   0x0B/0x0C = MAX_ANGLE_LIMIT (16-bit, little endian)
  // The servo silently clamps any WritePosEx target to this range,
  // so a wrong value here looks like "the motor stops short."
  int firstComma = line.indexOf(',');
  if (firstComma == -1)
  {
    Serial.println("ERROR,BAD_FORMAT");
    return;
  }

  String idText = line.substring(firstComma + 1);
  idText.trim();
  int motorId = idText.toInt();

  int minL = st.readByte(motorId, 9);
  int minH = st.readByte(motorId, 10);
  int maxL = st.readByte(motorId, 11);
  int maxH = st.readByte(motorId, 12);

  if (minL < 0 || minH < 0 || maxL < 0 || maxH < 0)
  {
    Serial.print("OK,LIMITS,");
    Serial.print(motorId);
    Serial.println(",-1,-1");
    return;
  }

  int minLimit = (minL & 0xFF) | ((minH & 0xFF) << 8);
  int maxLimit = (maxL & 0xFF) | ((maxH & 0xFF) << 8);

  Serial.print("OK,LIMITS,");
  Serial.print(motorId);
  Serial.print(",");
  Serial.print(minLimit);
  Serial.print(",");
  Serial.println(maxLimit);
}
