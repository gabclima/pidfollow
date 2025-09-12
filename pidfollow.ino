#include <QTRSensors.h>
#include <Arduino.h>
#include <BluetoothSerial.h>

QTRSensors qtr;

BluetoothSerial SerialBT; // Instância para comunicação Bluetooth

// Identificação dos sensores
const int SensorCount = 8;
uint16_t sensorValues[SensorCount];     

const int pwmFreq = 5000;       // Frequência de PWM
const int pwmResolution = 8;    // Resolução de 8 bits (valores de 0 a 255)

// Pinos para controle dos motores
const int pwmPinMotor1 = 19;    // Controle de PWM para o motor 1
const int dirPinMotor1 = 21;    // Direção do motor 1 (frente)
const int pwmPinMotor2 = 5;     // Controle de PWM para o motor 2
const int dirPinMotor2 = 18;    // Direção do motor 2 (frente)

const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

const int GOAL = 3500;

float Kp = 0.1; 
float Ki = 0.0; 
float Kd = 0.0;

int P, I, D;
double lastError = 0;

bool stopRequested = true; // Flag para parar o robô

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){13, 14, 27, 26, 25, 33, 32, 35}, SensorCount);
  // Configuração dos pinos de direção como saída
  pinMode(dirPinMotor1, OUTPUT);
  pinMode(dirPinMotor2, OUTPUT);

  // Configura o pino PWM para os dois motores
  ledcAttach(pwmPinMotor1, pwmFreq, pwmResolution);  // Motor 1
  ledcAttach(pwmPinMotor2, pwmFreq, pwmResolution);  // Motor 2

  // Inicializa as direções para frente
  digitalWrite(dirPinMotor1, LOW);  // Motor 1 para frente
  digitalWrite(dirPinMotor2, LOW);  // Motor 2 para frente

  ledcWrite(pwmPinMotor1, 0);   // Motor 1 parado
  ledcWrite(pwmPinMotor2, 0);   // Motor 2 parado 

  Serial.begin(9600); // Comunicação Serial padrão
  SerialBT.begin("RobotController"); // Nome do dispositivo Bluetooth

  Serial.println("Bluetooth iniciado");

  CalibrarSensor();
}

void loop() {
  // Verificar se há dados recebidos via Bluetooth
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n'); // Ler até encontrar uma nova linha
    command.trim(); // Remove espaços extras antes e depois do comando

    // Verificar se o comando é para somar ao Kp
    if (command.startsWith("Kp=")) {
      float value = command.substring(3).toFloat(); // Extrai o valor após "Kp="
      Kp = value; // Somar o valor ao Kp
      SerialBT.print("Novo valor de Kp: ");
      SerialBT.println(Kp);
    }

    // Verificar se o comando é para somar ao Ki
    else if (command.startsWith("Ki=")) {
      float value = command.substring(3).toFloat(); // Extrai o valor após "Ki="
      Ki = value; // Somar o valor ao Ki
      SerialBT.print("Novo valor de Ki: ");
      SerialBT.println(Ki);
    }

    // Verificar se o comando é para somar ao Kd
    else if (command.startsWith("Kd=")) {
      float value = command.substring(3).toFloat(); // Extrai o valor após "Kd="
      Kd = value; // Somar o valor ao Kd
      SerialBT.print("Novo valor de Kd: ");
      SerialBT.println(Kd);
    }

    // Verificar se o comando é para parar o robô
    else if (command == "stop") {
      stopRequested = true; // Ativa o comando de parada
      SerialBT.println("Robô parado");
    }

    else if (command == "go") {
      stopRequested = false; // Ativa o comando de parada
      SerialBT.println("Robô andano");
    }
  }

  if (stopRequested == true) {
    // Parar os motores
    ledcWrite(pwmPinMotor1, 0); 
    ledcWrite(pwmPinMotor2, 0);
  } else {
    // Lógica do controle PID
    int position = qtr.readLineWhite(sensorValues);
    int error = GOAL - position;

    P = error;
    I = I + error;
    D = error - lastError;
    lastError = error;

    int motorspeed = P*Kp + I*Ki + D*Kd;

    int leftSpeed = basespeeda + motorspeed;;
    int rightSpeed = basespeedb - motorspeed;

    if (leftSpeed > maxspeeda) {
      ledcWrite(pwmPinMotor1, leftSpeed = maxspeeda);
    }
    if (rightSpeed > maxspeedb) {
      ledcWrite(pwmPinMotor2, rightSpeed = maxspeedb);
    }
    if (leftSpeed < 0) {
      ledcWrite(pwmPinMotor1, leftSpeed = 0);
    }
    if (rightSpeed < 0) {
      ledcWrite(pwmPinMotor2, rightSpeed = 0);
    } 
    ledcWrite(pwmPinMotor2, 0);
    ledcWrite(pwmPinMotor1, 0);
      //Envia dados via Bluetooth
    SerialBT.print("Left Motor Speed: ");
    SerialBT.println(leftSpeed);
    SerialBT.print("Right Motor Speed: ");
    SerialBT.println(rightSpeed);
  }
}
  
void CalibrarSensor() {
    delay(500);

    for (int i = 0; i < 500; i++) {
        qtr.calibrate();
    }
    SerialBT.println("Calibrado pa krai");
    delay(2500);
}
