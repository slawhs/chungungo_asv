#include "motor.h"

Motor* Motor::instances[10] = {nullptr};

Motor::Motor(uint8_t enA, uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwmChannel)
    : _enA(enA), _in1(in1), _in2(in2), _encA(encA), _encB(encB), _pwmChannel(pwmChannel),
      _pulsos(0), _oldPulsos(0), _lastTime(0)
{
    // Guardar esta instancia para las interrupciones estáticas
    for (int i = 0; i < 10; i++) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            _instanceIndex = i;
            break;
        }
    }
}

void Motor::setup() {
    pinMode(_enA, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_encA, INPUT_PULLUP);
    pinMode(_encB, INPUT_PULLUP);

    ledcSetup(_pwmChannel, 5000, 8);
    ledcAttachPin(_enA, _pwmChannel);

    attachInterruptStatic(_encA, _contarPulsoA_static, CHANGE, this);
    attachInterruptStatic(_encB, _contarPulsoB_static, CHANGE, this);

    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);

    _lastTime = millis();
}

void Motor::set_speed(int velocidad) {
    int dir = 0;
    if (velocidad > 0) dir = 1;
    else if (velocidad < 0) dir = -1;

    velocidad = abs(velocidad);
    if (velocidad > 255) velocidad = 255;

    if (dir == 1) {
        ledcWrite(_pwmChannel, velocidad);
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    } else if (dir == -1) {
        ledcWrite(_pwmChannel, velocidad);
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else {
        // Frenado libre
        ledcWrite(_pwmChannel, 255);
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }
}

float Motor::get_rpm() {
    unsigned long now = millis();
    float razon = 1000.0 * PULSOS_POR_VUELTA / 60.0;
    float deltaT = (now - _lastTime);
    float rpm = (float)(_pulsos - _oldPulsos) * razon / deltaT;
    _oldPulsos = _pulsos;
    _lastTime = now;
    return rpm;
}

void Motor::_contarPulsoA() {
    if (digitalRead(_encA) == digitalRead(_encB)) _pulsos++;
    else _pulsos--;
}

void Motor::_contarPulsoB() {
    if (digitalRead(_encA) == digitalRead(_encB)) _pulsos--;
    else _pulsos++;
}

// --- Mapeo estático de interrupciones ---
void IRAM_ATTR Motor::_contarPulsoA_static() {
    for (int i = 0; i < 10; i++) {
        if (instances[i]) instances[i]->_contarPulsoA();
    }
}

void IRAM_ATTR Motor::_contarPulsoB_static() {
    for (int i = 0; i < 10; i++) {
        if (instances[i]) instances[i]->_contarPulsoB();
    }
}

void Motor::attachInterruptStatic(uint8_t pin, void (*func)(), int mode, Motor* instance) {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(pin, func, mode);
}
