#pragma once
#ifndef PinChangeInterrupt_H
#define PinChangeInterrupt_H


#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef EXTERNAL_NUM_INTERRUPTS
#define EXTERNAL_NUM_INTERRUPTS 3
#endif

#ifndef digitalPinToPCICR(p)
#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#endif

#ifndef digitalPinToPCICRbit(p)
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#endif

#ifndef digitalPinToPCMSK(p)
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#endif

typedef void (*void_function_pointer)(void);
static void nop(void) {}
static volatile void_function_pointer interrupt_function[EXTERNAL_NUM_INTERRUPTS] = {nop, nop, nop};

enum { PC_INT_0, PC_INT_1, PC_INT_2 };

class PinChangeInterrupt {
  public:
    PinChangeInterrupt();
    ~PinChangeInterrupt();
    void attachInterrupt(uint8_t pin, void (*userFunc)(void));
    void detachInterrupt();

  private:
    uint8_t interrupt_pin;
    uint8_t interrupt_pin_bit_mask;
    uint8_t *interrupt_pin_port;
};
#endif
