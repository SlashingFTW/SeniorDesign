#include <HardwareSerial.h>

HardwareSerial SerialMKS(1); // UART1 for MKS Servo42C

// Constants
const int PULSES_PER_REV = 3200;

void setup() {
  Serial.begin(115200);  // USB Serial for PC
  while (!Serial) delay(10);

  // MKS Servo UART: RX = GPIO20, TX = GPIO21
  SerialMKS.begin(38400, SERIAL_8N1, 20, 21);

  Serial.println("ESP32-S3 Ready.");
  Serial.println("Type 'e0 36' to read angle or 'move 90' to move to 90 degrees.");
}

float currentRPM = 100.0; // default RPM
bool ccw = false;         // default direction (optional)
uint8_t motorID = 0xE0; // Default motor ID


void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Set RPM
    if (input.startsWith("set rpm ")) {
      float rpm = input.substring(8).toFloat();
      if (rpm > 0 && rpm <= 1200) {
        currentRPM = rpm;
        Serial.print("✅ RPM set to ");
        Serial.println(currentRPM);
      } else {
        Serial.println("❌ Invalid RPM. Must be between 1 and 1200.");
      }
    }

    // Move with stored RPM
    else if (input.startsWith("move ")) {
      float angle = input.substring(5).toFloat();
      moveToAngle(angle, ccw, currentRPM);
      readCurrentAngle();
    }

    // Read angle
    else if (input.equalsIgnoreCase("read angle")) {
      readCurrentAngle();
    }

    else if (input.equalsIgnoreCase("home")) {
      moveHome();
    }

    else if (input.startsWith("set motor ")) {
      String idStr = input.substring(10); // get string after "set motor "
      idStr.toUpperCase();

      if (idStr.length() == 2 && idStr[0] == 'E') {
        int val = strtol(idStr.c_str() + 1, nullptr, 16);
        if (val >= 0x0 && val <= 0xF) {
          motorID = 0xE0 + val;
          Serial.print("✅ Motor ID set to ");
          Serial.println(idStr);
        } else {
          Serial.println("❌ Invalid motor ID. Must be E0–EF.");
        }
      } else {
        Serial.println("❌ Usage: set motor EX (e.g., E0, E1, E2...)");
      }
  }


    else {
      Serial.println("Invalid command. Try:");
      Serial.println("  set rpm <value>");
      Serial.println("  move <angle>");
      Serial.println("  read angle");
      Serial.println("  home");
    }
  }
}


// 🌀 Function to read current angle
void readCurrentAngle() {
  // Build and send command
  uint8_t cmd1 = motorID;
  uint8_t cmd2 = 0x36;
  uint8_t checksum = (cmd1 + cmd2) & 0xFF;

  SerialMKS.write(cmd1);
  SerialMKS.write(cmd2);
  SerialMKS.write(checksum);

  Serial.println("Command sent to read angle.");

  delay(20);

  // Expect 6-byte response
  uint8_t response[6];
  int i = 0;
  unsigned long start = millis();
  while (i < 6 && (millis() - start) < 100) {
    if (SerialMKS.available()) {
      response[i++] = SerialMKS.read();
    }
  }

  if (i == 6 && response[0] == motorID) {
    Serial.print("Raw response: ");
    for (int j = 0; j < 6; j++) {
      Serial.printf("%02X ", response[j]);
    }
    Serial.println();

    // Angle is in response[3] and response[4] (big-endian 16-bit)
    uint16_t raw_angle = (response[3] << 8) | response[4];
    float degrees = (raw_angle / 65535.0) * 360.0;

    Serial.print("Raw angle value: ");
    Serial.println(raw_angle);
    Serial.print("Angle in degrees: ");
    Serial.println(degrees, 2);
  } else {
    Serial.println("Invalid or no response from MKS.");
  }
}

// 🔁 Function to move to an angle
void moveToAngle(float angle_deg, bool ccw, float rpm) {
  uint32_t pulses = (angle_deg / 360.0) * PULSES_PER_REV;

  // Convert RPM to speed using: Speed = RPM / 9.375
  uint8_t speed = round(rpm / 9.375);
  if (speed > 127) speed = 127;

  // Direction + speed byte: MSB = direction
  uint8_t dir_speed = (ccw ? 0x00 : 0x80) | (speed & 0x7F);

  // Build command: E0 FD [dir+speed] [pulses MSB..LSB] [checksum]
  uint8_t cmd[7];
  cmd[0] = motorID;
  cmd[1] = 0xFD;
  cmd[2] = dir_speed;
  cmd[3] = (pulses >> 24) & 0xFF;
  cmd[4] = (pulses >> 16) & 0xFF;
  cmd[5] = (pulses >> 8) & 0xFF;
  cmd[6] = pulses & 0xFF;

  uint8_t tchk = 0;
  for (int i = 0; i < 7; i++) tchk += cmd[i];
  tchk &= 0xFF;

  for (int i = 0; i < 7; i++) SerialMKS.write(cmd[i]);
  SerialMKS.write(tchk);

  Serial.printf("Move command sent for %.2f° (%lu pulses), speed: %.2f RPM, dir: %s\n",
                angle_deg, pulses, rpm, ccw ? "CCW" : "CW");

  Serial.print("Command bytes: ");
  for (int i = 0; i < 7; i++) Serial.printf("%02X ", cmd[i]);
  Serial.printf("%02X\n", tchk);

  // Wait for run starting and run complete
  uint8_t expected1[3] = { motorID, 0x01, (uint8_t)((motorID + 0x01) & 0xFF) };
  uint8_t expected2[3] = { motorID, 0x02, (uint8_t)((motorID + 0x02) & 0xFF) };
  bool gotStart = false, gotDone = false;

  unsigned long start = millis();
  while ((millis() - start) < 5000) {
    if (SerialMKS.available() >= 3) {
      uint8_t r[3];
      r[0] = SerialMKS.read();
      r[1] = SerialMKS.read();
      r[2] = SerialMKS.read();

      if (r[0] == expected1[0] && r[1] == expected1[1] && r[2] == expected1[2]) {
        Serial.println("[MKS] Run starting...");
        gotStart = true;
      } else if (r[0] == expected2[0] && r[1] == expected2[1] && r[2] == expected2[2]) {
        Serial.println("[MKS] Run complete.");
        gotDone = true;
      } else {
        Serial.print("[MKS] Unknown response: ");
        for (int i = 0; i < 3; i++) Serial.printf("%02X ", r[i]);
        Serial.println();
      }

      if (gotStart && gotDone) break;
    }
  }

  if (!gotDone) {
    Serial.println("⚠️ Warning: move may not have completed.");
  }
}

//commang to go to zero
void moveHome() {
  uint8_t cmd[3] = {motorID, 0x94, 0x00};
  uint8_t tchk = (cmd[0] + cmd[1] + cmd[2]) & 0xFF;

  // Send command
  for (int i = 0; i < 3; i++) SerialMKS.write(cmd[i]);
  SerialMKS.write(tchk);

  // Print command sent in hex
  Serial.print("➡️  Sent home command: ");
  for (int i = 0; i < 3; i++) Serial.printf("%02X ", cmd[i]);
  Serial.printf("%02X\n", tchk);

  // Wait for 3-byte response: E0 01 rCHK
  unsigned long start = millis();
  while ((millis() - start) < 2000) {
    if (SerialMKS.available() >= 3) {
      uint8_t r0 = SerialMKS.read();
      uint8_t r1 = SerialMKS.read();
      uint8_t r2 = SerialMKS.read();

      Serial.print("⬅️  Received response: ");
      Serial.printf("%02X %02X %02X\n", r0, r1, r2);

      if (r0 == motorID && r1 == 0x01) { 
        Serial.println("✅ Home successful (received E0 01 rCHK)");
        return;
      } else {
        Serial.println("⚠️ Unexpected response bytes");
        return;
      }
    }
  }

  Serial.println("❌ No response from servo on home command.");
}



