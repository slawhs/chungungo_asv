#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
String device_name = "ESP32-BT-Slave";

// Motor 1
#define mot1_enA 27
#define mot1_in1 14
#define mot1_in2 26
#define enc1_ENCODER_A 23
#define enc1_ENCODER_B 22

//Motor 2
// #define mot2_enA 32
// #define mot2_in1 33
// #define mot2_in2 25
// #define enc2_ENCODER_A 21
// #define enc2_ENCODER_B 19


#define PULSOS_POR_VUELTA 16

volatile long enc1_pulsos = 0;
unsigned long ultimaMedicion = 0;
float mot1_rpm = 0;
int mot1_dir = 1; // 1 si es sentido horario, -1 si es antihorario, 0 si es frenado
int enc1_dir = 0;
long oldposition0 = 0;

String msg;

// Canal de PWM para ESP32
#define MOT1 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Variables para PID del primer motor
const float Kp = 1.0, Ki = 0.1, Kd = 0.1, Ts = 0.1;
float c_prev = 0, e_prev1 = 0, e_prev2 = 0;
float setpoint = 255;

void IRAM_ATTR contarPulsoA() {
    if (digitalRead(enc1_ENCODER_A) == digitalRead(enc1_ENCODER_B)) {
        enc1_pulsos++;
    } else {
        enc1_pulsos--;
    }
}
void IRAM_ATTR contarPulsoB() {
    if (digitalRead(enc1_ENCODER_A) == digitalRead(enc1_ENCODER_B)) {
        enc1_pulsos--;
    } else {
        enc1_pulsos++;
    }
}

void mot1_velocidad(int velocidad, int dir) {
    if (velocidad < 0) {
        velocidad = 0;
    } else if (velocidad > 255) {
        velocidad = 255;
    }
    // SerialBT.printf("velocidad: %d \n", velocidad);
    // SerialBT.printf("dir: %d \n", dir);
    if (dir == 1){
        if (velocidad < 185) {
            ledcWrite(MOT1, velocidad);
        } else {
            ledcWrite(MOT1, 255);
        }
        digitalWrite(mot1_in1, LOW);
        digitalWrite(mot1_in2, HIGH);            
    } else if (dir == -1){
        if (velocidad < 185) {
            ledcWrite(MOT1, velocidad);
        } else {
            ledcWrite(MOT1, 255);
        }
        digitalWrite(mot1_in1, HIGH);
        digitalWrite(mot1_in2, LOW);
    } else {
        // Frenado libre
        ledcWrite(MOT1, 255);
        digitalWrite(mot1_in1, LOW);
        digitalWrite(mot1_in2, LOW);
    }
}


void setup() {
    pinMode(mot1_enA, OUTPUT);
    pinMode(mot1_in1, OUTPUT);
    pinMode(mot1_in2, OUTPUT);
    pinMode(enc1_ENCODER_A, INPUT_PULLUP);
    pinMode(enc1_ENCODER_B, INPUT_PULLUP);

    // Inicializar PWM
    ledcSetup(MOT1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(mot1_enA, MOT1);

    attachInterrupt(enc1_ENCODER_A, contarPulsoA, CHANGE);
    attachInterrupt(enc1_ENCODER_B, contarPulsoB, CHANGE);

    digitalWrite(mot1_in1, LOW);
    digitalWrite(mot1_in2, HIGH);

    Serial.begin(115200);
    SerialBT.begin(device_name);  // Bluetooth device name

    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

    ultimaMedicion = millis();
}

void loop() {
    if (SerialBT.available()) {
        setpoint = SerialBT.readStringUntil('\n').toFloat();
    }

    delay(20);
    if (millis() - ultimaMedicion >= 100) {
        // --- mido rpm ---
        float rpm = 1000.0 * PULSOS_POR_VUELTA / 60.0; // RPM
        mot1_rpm = (float)(enc1_pulsos - oldposition0) * rpm / (millis() - ultimaMedicion); //RPM
        oldposition0 = enc1_pulsos;
        ultimaMedicion = millis();

        // --- PID discreto recursivo ---
        // float e0 = setpoint - mot1_rpm;
        // float a0 =  Kp + Ki*Ts + Kd/Ts;
        // float a1 = -Kp - 2.0*(Kd/Ts);
        // float a2 =  Kd/Ts;
        // float c0 = c_prev + a0*e0 + a1*e_prev1 + a2*e_prev2;
        // e_prev2 = e_prev1; e_prev1 = e0; c_prev = c0;

        int pwm = setpoint;
        if (pwm > 0){
            mot1_dir = 1; // sentido horario
        } else if (pwm < 0){
            mot1_dir = -1; // sentido antihorario
        } else {
            mot1_dir = 0; // frenado
        }
        pwm = abs(pwm);
        mot1_velocidad(pwm, mot1_dir);

        // --- debug por BT ---
        SerialBT.printf("SP: %.1f  RPM: %.2f  PWM: %d\n", setpoint, mot1_rpm, pwm);
    }
}
