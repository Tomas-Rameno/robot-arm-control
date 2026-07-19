/*
  RobotArmBluetooth.ino
  ----------------------------------------------------------
  Ovládání 6-osého robotického ramene (serva 0-180°) pomocí
  ESP32 a Bluetooth Classic (SPP) - ovládání appkou z telefonu.

  HARDWARE
    - ESP32 (klasický - Classic Bluetooth nemají ESP32-S2/S3/C3/C6,
      ty mají jen BLE; pokud máš některou z těchto verzí, napiš a
      přepíšu kód na BLE)
    - 6x servo 0-180°
    - Serva napájet ze SAMOSTATNÉHO zdroje 5-6V (klidně 3-5A podle
      velikosti serv) - NE z 5V/3.3V pinu ESP32, ten to neutáhne!
      GND zdroje a ESP32 musí být propojené (společná zem).

  ZAPOJENÍ SIGNÁLOVÝCH VODIČŮ (uprav v poli JOINT_PIN níže dle potřeby)
    Base     (základna)          -> GPIO 13
    Shoulder (rameno/plece)      -> GPIO 14
    Elbow    (loket)             -> GPIO 27
    Wrist    (zápěstí náklon)    -> GPIO 26
    WristRot (zápěstí rotace)    -> GPIO 25
    Gripper  (drapák)            -> GPIO 33

  KNIHOVNA
    Library Manager -> nainstaluj "ESP32Servo" (autor: Kevin Harrington / madhephaestus)

  KOMUNIKAČNÍ PROTOKOL (text přes Bluetooth, řádek ukončený \n)
    B<úhel>   základna,            např. B90
    S<úhel>   rameno
    E<úhel>   loket
    W<úhel>   zápěstí náklon
    R<úhel>   zápěstí rotace
    G<úhel>   drapák
    Víc příkazů na řádek odděl čárkou:   B90,S45,E120
    H         přejede do výchozí (home) pozice
    ?         vypíše aktuální úhly všech kloubů (STATUS B90,S90,...)

  TESTOVÁNÍ
    Android appka "Serial Bluetooth Terminal" (Kai Morich, zdarma) -
    spáruj telefon se zařízením "RobotArm" a posílej příkazy ručně,
    než si uděláš vlastní appku. Pozor: klasický SPP Bluetooth
    nefunguje na iPhone (iOS ho blokuje pro necertifikovaná zařízení).
*/

#include "BluetoothSerial.h"
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth neni povoleny - zkontroluj Tools > Partition Scheme v Arduino IDE.
#endif

BluetoothSerial SerialBT;

const int JOINT_COUNT = 6;

// Písmeno příkazu | GPIO pin | min úhel | max úhel | home pozice
const char JOINT_CODE[JOINT_COUNT] = {'B',  'S',  'E',  'W',  'R',  'G'};
const int  JOINT_PIN[JOINT_COUNT]  = { 13,   14,   27,   26,   25,   33};
const int  JOINT_MIN[JOINT_COUNT]  = {  0,    0,    0,    0,    0,    0};
const int  JOINT_MAX[JOINT_COUNT]  = {180,  180,  180,  180,  180,   90};
const int  JOINT_HOME[JOINT_COUNT] = { 90,   90,   90,   90,   90,   30};

Servo servos[JOINT_COUNT];
int currentAngle[JOINT_COUNT];
int targetAngle[JOINT_COUNT];

String inputBuffer = "";

const unsigned long STEP_DELAY_MS = 15;  // pauza mezi kroky plynulého pohybu (víc = pomalejší a plynulejší)
const int STEP_SIZE_DEG = 1;             // o kolik stupňů se kloub posune za jeden krok
unsigned long lastStepTime = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("RobotArm");
  Serial.println("Bluetooth spusten - sparuj telefon se zarizenim 'RobotArm'.");

  // ESP32Servo potřebuje alokovat časovače pro přesné PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < JOINT_COUNT; i++) {
    servos[i].setPeriodHertz(50);               // standardních 50Hz pro hobby serva
    servos[i].attach(JOINT_PIN[i], 500, 2400);  // min/max pulz v mikrosekundách - uprav, pokud servo na krajích bzučí
    currentAngle[i] = JOINT_HOME[i];
    targetAngle[i]  = JOINT_HOME[i];
    servos[i].write(currentAngle[i]);
  }
}

void loop() {
  readBluetoothCommands();

  if (millis() - lastStepTime >= STEP_DELAY_MS) {
    lastStepTime = millis();
    stepServosTowardTarget();
  }
}

// Čte příchozí bajty z Bluetooth a skládá je do řádku až po \n
void readBluetoothCommands() {
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// Posune každý kloub o kousek blíž k cílovému úhlu (plynulý pohyb, menší proudové špičky)
void stepServosTowardTarget() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    if (currentAngle[i] == targetAngle[i]) continue;
    if (currentAngle[i] < targetAngle[i]) {
      currentAngle[i] = min(currentAngle[i] + STEP_SIZE_DEG, targetAngle[i]);
    } else {
      currentAngle[i] = max(currentAngle[i] - STEP_SIZE_DEG, targetAngle[i]);
    }
    servos[i].write(currentAngle[i]);
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "?") {
    sendStatus();
    return;
  }
  if (cmd.equalsIgnoreCase("H")) {
    goHome();
    return;
  }

  // Víc příkazů na řádek oddělených čárkou, např. "B90,S45,E120"
  int start = 0;
  while (start < (int)cmd.length()) {
    int comma = cmd.indexOf(',', start);
    String part = (comma == -1) ? cmd.substring(start) : cmd.substring(start, comma);
    applySingleCommand(part);
    if (comma == -1) break;
    start = comma + 1;
  }
}

void applySingleCommand(String part) {
  part.trim();
  if (part.length() < 2) return;

  char code = toupper(part.charAt(0));
  int angle = part.substring(1).toInt();

  for (int i = 0; i < JOINT_COUNT; i++) {
    if (JOINT_CODE[i] == code) {
      angle = constrain(angle, JOINT_MIN[i], JOINT_MAX[i]);
      targetAngle[i] = angle;
      SerialBT.printf("OK %c%d\n", code, angle);
      return;
    }
  }
  SerialBT.printf("ERR neznamy kloub '%c'\n", code);
}

void goHome() {
  for (int i = 0; i < JOINT_COUNT; i++) {
    targetAngle[i] = JOINT_HOME[i];
  }
  SerialBT.println("OK HOME");
}

void sendStatus() {
  String status = "STATUS ";
  for (int i = 0; i < JOINT_COUNT; i++) {
    status += JOINT_CODE[i];
    status += currentAngle[i];
    if (i < JOINT_COUNT - 1) status += ",";
  }
  SerialBT.println(status);
}
