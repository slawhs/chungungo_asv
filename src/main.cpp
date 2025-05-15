#include <Arduino.h>
//#include "BluetoothSerial.h"

// BluetoothSerial SerialBT;
// String device_name = "ESP32-BT-Slave";

// Motor 1
#define mot1_enA 27
#define mot1_in1 14
#define mot1_in2 26
#define enc1_ENCODER_A 23
#define enc1_ENCODER_B 22

//Motor 2
#define mot2_enA 32
#define mot2_in1 33
#define mot2_in2 25
#define enc2_ENCODER_A 21
#define enc2_ENCODER_B 19


#define PULSOS_POR_VUELTA 32

volatile long enc1_pulsos = 0;
volatile long enc2_pulsos = 0;
unsigned long ultimaMedicion = 0;
float mot1_rpm = 0;
float mot2_rpm = 0;
int vel1, dir1, vel2, dir2;


String msg;

// Canal de PWM para ESP32
#define MOT1 0
#define MOT2 1
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define MAX_VEL 220

// Variables para PID del primer motor
const float Kp = 0.3, Ki = 0.01, Kd = 0.0, Ts = 0.1;
float c_prev = 0, e_prev1 = 0, e_prev2 = 0;
float setpoint = 0;

void IRAM_ATTR contarPulsoA1() {
    if (digitalRead(enc1_ENCODER_A) != digitalRead(enc1_ENCODER_B)) {
        enc1_pulsos++;
    } else {
        enc1_pulsos--;
    }
}

void IRAM_ATTR contarPulsoA2() {
    if (digitalRead(enc2_ENCODER_A) != digitalRead(enc2_ENCODER_B)) {
        enc2_pulsos++;
    } else {
        enc2_pulsos--;
    }
}

void mot1_velocidad(int velocidad, int direccion) {
    if (velocidad < 0) {
        velocidad = 0;
    } else if (velocidad > 100) {
        velocidad = 100;
    }
    int vel_map = map(velocidad, 0, 100, MAX_VEL, 0); // 0 -> 185 and 100 -> 0
    //int vel_map = velocidad;
    ledcWrite(MOT1, vel_map);

    if (vel_map < MAX_VEL) {
        if (direccion != 0) {
            ledcWrite(MOT1, vel_map);
        } else {
            ledcWrite(MOT1, 255);
        }
    } else {
        ledcWrite(MOT1, 255);
    }
}
void mot1_direccion(int dir) {
    if (dir == 1) {
        digitalWrite(mot1_in1, LOW);
        digitalWrite(mot1_in2, HIGH);
    } else if (dir == -1) {
        digitalWrite(mot1_in1, HIGH);
        digitalWrite(mot1_in2, LOW);
    } else if (dir == 0) {
        // Frenado libre
        digitalWrite(mot1_in1, LOW);
        digitalWrite(mot1_in2, LOW);
    }
}

void mot2_velocidad(int velocidad, int direccion) {
    if (velocidad < 0) {
        velocidad = 0;
    } else if (velocidad > 100) {
        velocidad = 100;
    }
    int vel_map = map(velocidad, 0, 100, MAX_VEL, 0); // 0 -> 185 and 100 -> 0
    // int vel_map = velocidad;
    ledcWrite(MOT2, vel_map);

    
    if (vel_map < MAX_VEL) {
        if (direccion != 0) {
            ledcWrite(MOT2, vel_map);
        } else {
            ledcWrite(MOT2, 255);
        }
    } else {
        ledcWrite(MOT2, 255);
    }
}
void mot2_direccion(int dir) {
    if (dir == 1) {
        digitalWrite(mot2_in1, LOW);
        digitalWrite(mot2_in2, HIGH);
    } else if (dir == -1) {
        digitalWrite(mot2_in1, HIGH);
        digitalWrite(mot2_in2, LOW);
    } else if (dir == 0) {
        // Frenado libre
        digitalWrite(mot2_in1, LOW);
        digitalWrite(mot2_in2, LOW);
    }
}
void setup() {
    pinMode(mot1_enA, OUTPUT);
    pinMode(mot1_in1, OUTPUT);
    pinMode(mot1_in2, OUTPUT);
    pinMode(enc1_ENCODER_A, INPUT_PULLUP);
    pinMode(enc1_ENCODER_B, INPUT_PULLUP);

    pinMode(mot2_enA, OUTPUT);
    pinMode(mot2_in1, OUTPUT);
    pinMode(mot2_in2, OUTPUT);
    pinMode(enc2_ENCODER_A, INPUT_PULLUP);
    pinMode(enc2_ENCODER_B, INPUT_PULLUP);
    

        
    // Configurar el canal de PWM para el motor 1
    ledcSetup(MOT2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(mot2_enA, MOT2);

    // Inicializar PWM
    ledcSetup(MOT1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(mot1_enA, MOT1);

    attachInterrupt(enc1_ENCODER_A, contarPulsoA1, RISING);
    attachInterrupt(enc2_ENCODER_A, contarPulsoA2, RISING);

    digitalWrite(mot1_in1, LOW);
    digitalWrite(mot1_in2, HIGH);

    digitalWrite(mot2_in1, LOW);
    digitalWrite(mot2_in2, HIGH);

    Serial.begin(115200);
    //SerialBT.begin(device_name);  // Bluetooth device name

    Serial.println("The device started");
    //Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

    ultimaMedicion = millis();
}

float control_pid(float set_point, float mot_rpm) {
    // --- PID discreto recursivo ---
    float e0 = set_point - mot_rpm;
    float a0 =  Kp + Ki*Ts + Kd/Ts;
    float a1 = -Kp - 2.0*(Kd/Ts);
    float a2 =  Kd/Ts;
    float c0 = c_prev + a0*e0 + a1*e_prev1 + a2*e_prev2;
    e_prev2 = e_prev1; e_prev1 = e0; c_prev = c0;

    return c0;
}

void loop() {
    if (Serial.available()) {
        msg = Serial.readStringUntil('\n');
        sscanf(msg.c_str(), "%d,%d,%d,%d", &vel1, &dir1, &vel2, &dir2);
    }

    delay(20);
    if (millis() - ultimaMedicion >= 100) {
        // --- mido rpm ---
        mot1_rpm = (float)((enc1_pulsos * 600 / PULSOS_POR_VUELTA)/9.7);
        mot2_rpm = (float)((enc2_pulsos * 600 / PULSOS_POR_VUELTA)/9.7);
        enc1_pulsos = 0;
        enc2_pulsos = 0;

        // --- PID discreto recursivo ---
        //float c0 = control_pid(setpoint, mot1_rpm);
        //int pwm = map(abs(c0), 0, 5000, 0, 100); // PWM de 0 a 255
    
        mot1_velocidad(vel1, dir1);
        mot1_direccion(dir1);

        mot2_velocidad(vel2, dir2);
        mot2_direccion(dir2);

        
        // --- debug por BT ---
        // float enc2_pulsos_float = (float)enc2_pulsos;
        //SerialBT.printf("pulsos2: %.2f  vueltas2: %.2f rpm: %.1f \n", enc2_pulsos_float, vueltas2, mot2_rpm);
        Serial.printf("%.2f | %.2f\n", mot1_rpm, mot2_rpm);
        ultimaMedicion = millis();
        //SerialBT.printf("SP: %.1f  RPM: %.2f  PWM: %d  c0: %.2f  dir: %d \n", setpoint, mot1_rpm, pwm, c0, mot1_dir);
    }
}
