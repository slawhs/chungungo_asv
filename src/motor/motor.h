#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t enA, uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwmChannel);

    void setup();
    void set_speed(int velocidad);
    float get_rpm();

private:
    uint8_t _enA, _in1, _in2, _encA, _encB, _pwmChannel;

    volatile long _pulsos;
    long _oldPulsos;
    unsigned long _lastTime;

    static const int PULSOS_POR_VUELTA = 16;

    void _configurarInterrupciones();
    static void _contarPulsoA_static();
    static void _contarPulsoB_static();

    void _contarPulsoA();
    void _contarPulsoB();

    static Motor* instances[10];
    static void attachInterruptStatic(uint8_t pin, void (*func)(), int mode, Motor* instance);

    int _instanceIndex;
};

#endif
