#include <Arduino.h>
// #include "BluetoothSerial.h"

// BluetoothSerial SerialBT;
// String device_name = "ESP32-BT-Slave";

// Motor 1
#define mot1_enA 27
#define mot1_in1 14
#define mot1_in2 26
#define enc1_ENCODER_A 22
#define enc1_ENCODER_B 23

//Motor 2
#define mot2_enA 32
#define mot2_in1 33
#define mot2_in2 25
#define enc2_ENCODER_A 19
#define enc2_ENCODER_B 21


#define PULSOS_POR_VUELTA 310

volatile long enc1_pulsos = 0;
volatile long enc2_pulsos = 0;
unsigned long medicionActual = 0;
unsigned long ultimaMedicion = 0;
unsigned long ultimaMedicionFeedback = 0;
float mot1_rpm = 0;
float mot2_rpm = 0;
int vel1, dir1, vel2, dir2;
bool ledState = false;

int vel1_map = 0;
int vel2_map = 0;



String msg;

typedef struct Motores {
  float rpm_values[4];
  float rpm_avg;
};

struct Motores motor1 = {
  { 0.0, 0.0, 0.0, 0.0 },  // rpm_values
  0.0                      // rpm_avg
};
struct Motores motor2 = {
  { 0.0, 0.0, 0.0, 0.0 },  // rpm_values
  0.0                      // rpm_avg
};

void add_rpm_value(Motores &motor, float new_val) {
  float sum = 0.0;
  for (int i = 0; i < 3; i++) {
    motor.rpm_values[i] = motor.rpm_values[i + 1];
  }
  motor.rpm_values[3] = new_val;

  for (int i = 0; i < 4; i++) {
    sum += motor.rpm_values[i];
  }
  motor.rpm_avg = sum / 4.0;
}


#define MIN_VEL 252
#define MAX_VEL 1
#define PULSOS_1_VUELTA 320

// Variables para PID del primer motor
const float Ts = 0.01;
const float Kp1 = 0.01, Ki1 = 0.0, Kd1 = 0.0;  //Kp1 = 1.5 , Ki1 = 0.03, Kd1 = 0.0015;
float c1_prev = 0, e1_prev1 = 0, e1_prev2 = 0;
float mot1_setpoint = 0;

// Variables para PID del segundo motor
const float Kp2 = 0.01, Ki2 = 0.0, Kd2 = 0.0;
float c2_prev = 0, e2_prev1 = 0, e2_prev2 = 0;
float mot2_setpoint = 0;

void IRAM_ATTR contarPulsoA1() {
  if (digitalRead(enc1_ENCODER_A) != digitalRead(enc1_ENCODER_B)) {
    enc1_pulsos++;
  } else {
    enc1_pulsos--;
  }
}

// void IRAM_ATTR contarPulsoB1() {
//   if (digitalRead(enc1_ENCODER_B) == digitalRead(enc1_ENCODER_A)) {
//     enc1_pulsos--;
//   } else {
//     enc1_pulsos++;
//   }
// }

void IRAM_ATTR contarPulsoA2() {
  if (digitalRead(enc2_ENCODER_A) != digitalRead(enc2_ENCODER_B)) {
    enc2_pulsos++;
  } else {
    enc2_pulsos--;
  }
}

int pwmInvertidoConZonaMuerta(int input) {
  const int input_min = 0;
  const int input_max = 1000;
  const int pwm_min = 1;    // Velocidad máxima (PWM más bajo por inversión)
  const int pwm_max = 252;  // Velocidad mínima (PWM más alto, aún en zona útil)

  input = constrain(input, input_min, input_max);

  float scale = (float)(input_max - input) / (input_max - input_min);
  return (int)(scale * (pwm_max - pwm_min) + pwm_min);
}

void mot1_velocidad(int velocidad, int direccion) {
  // if (velocidad < 0) {
  //   velocidad = 0;
  // } else if (velocidad > 100) {
  //   velocidad = 100;
  // }
  vel1_map = pwmInvertidoConZonaMuerta(velocidad);  // 0 -> 185 and 100 -> 0

  analogWrite(mot1_enA, vel1_map);

  //   if (vel1_map > MIN_VEL) {
  //     if (direccion != 0) {
  //       analogWrite(mot1_enA, vel1_map);
  //     } else {
  //       analogWrite(mot1_enA, 255);
  //     }
  //   } else {
  //     analogWrite(mot1_enA, 255);
  //   }
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
  vel2_map = pwmInvertidoConZonaMuerta(velocidad);  // 0 -> 185 and 100 -> 0
  // int vel_map = velocidad;
  //analogWrite(mot2_enA, vel_map);


  if (vel2_map < MIN_VEL) {
    if (direccion != 0) {
      analogWrite(mot2_enA, vel2_map);
    } else {
      analogWrite(mot2_enA, 255);
    }
  } else {
    analogWrite(mot2_enA, 255);
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

// float control_pid1(float set_point, float mot_rpm) {
//   // --- PID discreto recursivo ---
//   float e0 = set_point - mot_rpm;
//   float a0 = Kp1 + Ki1 * Ts + Kd1 / Ts;
//   float a1 = -Kp1 - 2.0 * (Kd1 / Ts);
//   float a2 = Kd1 / Ts;
//   float c0 = c1_prev + a0 * e0 + a1 * e1_prev1 + a2 * e1_prev2;
//   e1_prev2 = e1_prev1;
//   e1_prev1 = e0;
//   c1_prev = c0;
//   if (c0 > 100.0) {
//     c0 = 100.0;
//   }
//   if (c0 < -100.0) {
//     c0 = -100.0;
//   }
//   return c0;
// }

// float control_pid2(float set_point, float mot_rpm) {
//   // --- PID discreto recursivo ---
//   float e0 = set_point - mot_rpm;
//   float a0 = Kp2 + Ki2 * Ts + Kd2 / Ts;
//   float a1 = -Kp2 - 2.0 * (Kd2 / Ts);
//   float a2 = Kd2 / Ts;
//   float c0 = c2_prev + a0 * e0 + a1 * e2_prev1 + a2 * e2_prev2;
//   e2_prev2 = e2_prev1;
//   e2_prev1 = e0;
//   c2_prev = c0;
//   if (c0 > 100.0) {
//     c0 = 100.0;
//   }
//   if (c0 < -100.0) {
//     c0 = -100.0;
//   }

//   return c0;
// }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
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

  // Inicializar PWM
  attachInterrupt(enc1_ENCODER_A, contarPulsoA1, RISING);
  //attachInterrupt(enc1_ENCODER_B, contarPulsoB1, CHANGE);
  attachInterrupt(enc2_ENCODER_A, contarPulsoA2, RISING);

  digitalWrite(mot1_in1, LOW);
  digitalWrite(mot1_in2, HIGH);

  digitalWrite(mot2_in1, LOW);
  digitalWrite(mot2_in2, HIGH);

  Serial.begin(115200);
  //SerialBT.begin(device_name);  // Bluetooth device name

  //Serial.println("The device started");
  //Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  ultimaMedicion = millis();
}

void loop() {
  if (Serial.available()) {
    msg = Serial.readStringUntil('\n');
    sscanf(msg.c_str(), "%f,%f", &mot1_setpoint, &mot2_setpoint);
    // mot1_setpoint = msg.toInt();
    // mot2_setpoint = msg.toInt();
    digitalWrite(LED_BUILTIN, !ledState);  // Toggle LED state
    ledState = !ledState;                  // Toggle LED state
  }

  delay(0.1);

  /*
  Podemos cambiar la funcion de Serial.Available() por un while y obtener lo por caracter a caracter
  Supuestamente asi es mas eficiente y toma menos tiempo la funcion.
  No es importante ahora mismo.
  */

  medicionActual = millis();
  if (medicionActual - ultimaMedicion >= Ts * 1000) {  // Ts* 1000: Sample time en milisegundos
    // --- mido rpm ---
    float dt = (medicionActual - ultimaMedicion) / 1000.0;  // Tiempo transcurrido en segundos

    float mot1_vueltas = (float)enc1_pulsos / PULSOS_POR_VUELTA;
    mot1_rpm = (mot1_vueltas * 60.0) / (dt * 5);
    enc1_pulsos = 0;
    add_rpm_value(motor1, mot1_rpm);  // Añadir el valor actual de RPM al array y calcular el promedio


    float mot2_vueltas = (float)enc2_pulsos / PULSOS_POR_VUELTA;
    mot2_rpm = (mot2_vueltas * 60.0) / dt;
    enc2_pulsos = 0;
    add_rpm_value(motor2, mot2_rpm);

    // --- PID discreto recursivo ---
    // float c1 = control_pid1(mot1_setpoint, mot1_rpm);
    float c1 = mot1_setpoint;
    int pwm1 = abs(c1);
    if (c1 > 0) {
      dir1 = 1;  // Forward direction
    } else if (c1 < 0) {
      dir1 = -1;  // Reverse direction
    } else {
      dir1 = 0;  // Stop
    }
    mot1_velocidad(pwm1, dir1);
    mot1_direccion(dir1);

    // float c2 = control_pid2(mot2_setpoint, mot2_rpm);
    float c2 = mot2_setpoint;
    int pwm2 = abs(c2);
    if (c2 > 0) {
      dir2 = 1;  // Forward direction
    } else if (c2 < 0) {
      dir2 = -1;  // Reverse direction
    } else {
      dir2 = 0;  // Stop
    }
    mot2_velocidad(pwm2, dir2);
    mot2_direccion(dir2);
    ultimaMedicion = medicionActual;
    // Serial.println(millis());
  }

  if (medicionActual - ultimaMedicionFeedback >= 100) {  // Enviar feedback cada 100 ms
    ultimaMedicionFeedback = medicionActual;
    // Serial.print("vel1_mapeada");
    // Serial.print(vel1_map);
    // Serial.print(",");
    // Serial.print("c1:");
    // Serial.print(c1);
    // Serial.print(",");
    Serial.print("RPM1:");
    Serial.print(mot1_rpm);
    Serial.print(",");
    Serial.print("Setpoint1:");
    Serial.print(mot1_setpoint);
    Serial.print(",");
    Serial.print("Encoder1:");
    Serial.print(enc1_pulsos);
    Serial.print(",");

    // Serial.print("c2:");
    // Serial.print(c2);
    // Serial.print(",");
    Serial.print("RPM2:");
    Serial.print(mot2_rpm);
    Serial.print(",");
    Serial.print("Setpoint2:");
    Serial.println(mot2_setpoint);

    // Feedback para la Raspberry pi 4B
    // Serial.printf("%.2f|%.2f\n", mot1_rpm, mot2_rpm);
  }
}