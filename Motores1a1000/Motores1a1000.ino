#include <Arduino.h>
// This project is designed to run on the ESP32 platform.

// Motor 1
#define mot1_enA 27
#define mot1_in1 14
#define mot1_in2 26
#define enc1_ENCODER_A 22
#define enc1_ENCODER_B 23
#define mot1_agua 34

// Motor 2
#define mot2_enA 32
#define mot2_in1 33
#define mot2_in2 25
#define enc2_ENCODER_A 19
#define enc2_ENCODER_B 21
#define mot2_agua 35

#define relay_pin 17
#define PULSOS_POR_VUELTA 310

volatile long enc1_pulsos = 0;
volatile long enc2_pulsos = 0;
unsigned long medicionActual = 0;
unsigned long ultimaMedicion = 0;
unsigned long ultimaMedicionFeedback = 0;
float mot1_rpm = 0;
float mot2_rpm = 0;
bool ledState = false;
const int water_threshold = 500;

String msg;

typedef struct Motores {
    float rpm_values[6];
    float rpm_avg;
    int agua;
    const float Kp;
    const float Ki;
    const float Kd;
    float setpoint;
    float e_prev1;
    float e_prev2;
    float c_prev;
};

struct Motores motor1 = {
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // rpm_values
    0.0,                               // rpm_avg
    0,                                 // Agua
    1.0,                               // Kp
    0.0,                               // Ki
    0.0,                               // Kd
    0.0,                               // setpoint
    0.0,                               // e_prev1
    0.0,                               // e_prev2
    0.0                                // c_prev
};
struct Motores motor2 = {
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // rpm_values
    0.0,                               // rpm_avg
    0,                                 // Agua
    1.0,                               // Kp
    0.0,                               // Ki
    0.0,                               // Kd
    0.0,                               // setpoint
    0.0,                               // e_prev1
    0.0,                               // e_prev2
    0.0                                // c_prev
};

void add_rpm_value(Motores &motor, float new_val) {
    float sum = 0.0;
    for (int i = 0; i < 5; i++) {
        motor.rpm_values[i] = motor.rpm_values[i + 1];
    }
    motor.rpm_values[5] = new_val;

    for (int i = 0; i < 6; i++) {
        sum += motor.rpm_values[i];
    }
    motor.rpm_avg = sum / 6.0;
}

#define MIN_VEL 252
#define MAX_VEL 1
#define PULSOS_1_VUELTA 320

const float Ts = 0.01;  // Sample time in seconds


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

void feedbackRPi() {
    // Enviar datos de RPM y setpoint a la Raspberry Pi
    Serial.print(motor1.rpm_avg);
    Serial.print("|");
    Serial.println(motor2.rpm_avg);
}

void feedbackPC() {
    Serial.print("RPM1:");
    Serial.print(mot1_rpm);
    Serial.print(",");
    Serial.print("RPM1_avg:");
    Serial.print(motor1.rpm_avg);
    Serial.print(",");
    Serial.print("Agua1:");
    Serial.print(motor1.agua);
    Serial.print(",");
    Serial.print("Setpoint1:");
    Serial.print(motor1.setpoint);

    Serial.print(",");
    Serial.print("RPM2:");
    Serial.print(mot2_rpm);
    Serial.print(",");
    Serial.print("RPM2_avg:");
    Serial.print(motor2.rpm_avg);
    Serial.print(",");
    Serial.print("Agua2:");
    Serial.print(motor2.agua);
    Serial.print(",");
    Serial.print("Setpoint2:");
    Serial.println(motor2.setpoint);
}

void ComprobarAgua() {
    motor1.agua = analogRead(mot1_agua);
    motor2.agua = analogRead(mot2_agua);

    if (motor1.agua > water_threshold) {
        digitalWrite(relay_pin, HIGH);
    }
    if (motor2.agua > water_threshold) {
        digitalWrite(relay_pin, HIGH);
    }
}

int pwmInvertidoConZonaMuerta(int input) {
    const int input_min = 0;
    const int input_max = 1050;
    const int pwm_min = 174;  // Velocidad máxima (PWM más bajo por inversión) NO CAMBIAR!!!!
    const int pwm_max = 244;  // Velocidad mínima (PWM más alto, aún en zona útil)

    input = constrain(input, input_min, input_max);

    float scale = (float)(input_max - input) / (input_max - input_min);
    int result = (int)(scale * (pwm_max - pwm_min) + pwm_min);

    if (result >= pwm_max) {
        return 255;
    }
    if (result < 174) {
        return 174;
    }
    return result;
}

void mot1_velocidad(int velocidad) {
    int vel1_map = pwmInvertidoConZonaMuerta(velocidad);  // 0 -> 185 and 100 -> 0
    analogWrite(mot1_enA, vel1_map);
}

void mot1_direccion(float c1) {
    if (c1 > 0) {
        digitalWrite(mot1_in1, LOW);
        digitalWrite(mot1_in2, HIGH);
    } else if (c1 < 0) {
        digitalWrite(mot1_in1, HIGH);
        digitalWrite(mot1_in2, LOW);
    } else {
        digitalWrite(mot1_in1, LOW);
        digitalWrite(mot1_in2, LOW);
    }
}

void mot2_velocidad(int velocidad) {
    int vel2_map = pwmInvertidoConZonaMuerta(velocidad);  // 0 -> 185 and 100 -> 0
    analogWrite(mot2_enA, vel2_map);
}

void mot2_direccion(float c2) {
    if (c2 > 0) {
        digitalWrite(mot2_in1, LOW);
        digitalWrite(mot2_in2, HIGH);
    } else if (c2 < 0) {
        digitalWrite(mot2_in1, HIGH);
        digitalWrite(mot2_in2, LOW);
    } else {
        digitalWrite(mot2_in1, LOW);
        digitalWrite(mot2_in2, LOW);
    }
}

float control_pid(Motores &motor) {
    // --- PID discreto recursivo ---
    float e0 = motor.setpoint - motor.rpm_avg;
    float a0 = motor.Kp + motor.Ki * Ts + motor.Kd / Ts;
    float a1 = -motor.Kp - 2.0 * (motor.Kd / Ts);
    float a2 = motor.Kd / Ts;
    float c0 = motor.c_prev + a0 * e0 + a1 * motor.e_prev1 + a2 * motor.e_prev2;
    motor.e_prev2 = motor.e_prev1;
    motor.e_prev1 = e0;
    motor.c_prev = c0;
    return c0;
}

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

    //pinMode(relay_pin, OUTPUT);

    // Inicializar PWM
    attachInterrupt(enc1_ENCODER_A, contarPulsoA1, RISING);
    attachInterrupt(enc2_ENCODER_A, contarPulsoA2, RISING);

    digitalWrite(mot1_in1, LOW);
    digitalWrite(mot1_in2, HIGH);

    digitalWrite(mot2_in1, LOW);
    digitalWrite(mot2_in2, HIGH);

    Serial.begin(115200);
    ultimaMedicion = millis();
}

void loop() {
    if (Serial.available()) {
        msg = Serial.readStringUntil('\n');
        sscanf(msg.c_str(), "%f,%f", &motor1.setpoint, &motor2.setpoint);
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
        noInterrupts();
        float dt = (medicionActual - ultimaMedicion) / 1000.0;  // Tiempo transcurrido en segundos
        float mot1_vueltas = (float)enc1_pulsos / PULSOS_POR_VUELTA;
        mot1_rpm = (mot1_vueltas * 60.0) / dt;
        float mot2_vueltas = (float)enc2_pulsos / PULSOS_POR_VUELTA;
        mot2_rpm = (mot2_vueltas * 60.0) / dt;
        // mot1_rpm += 0.1;
        // mot2_rpm += 0.1;
        enc2_pulsos = 0;
        enc1_pulsos = 0;
        interrupts();
        add_rpm_value(motor1, mot1_rpm);  // Añadir el valor actual de RPM al array y calcular el promedio
        add_rpm_value(motor2, mot2_rpm);

        // --- PID discreto recursivo ---
        // float c1 = control_pid(motor1);
        float c1 = motor1.setpoint;
        int pwm1 = abs(c1);
        mot1_velocidad(pwm1);
        mot1_direccion(c1);

        // float c2 = control_pid(motor2);
        float c2 = motor2.setpoint;
        int pwm2 = abs(c2);
        mot2_velocidad(pwm2);
        mot2_direccion(c2);

        /* LECTURA SENSOR AGUA*/
        //ComprobarAgua();
        ultimaMedicion = medicionActual;

        /* FEEDBACK */
        //feedbackPC();
        feedbackRPi();
    }
}