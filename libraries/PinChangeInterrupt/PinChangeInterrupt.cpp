#include "PinChangeInterrupt.h"

#define _ISR(vect, interrupt) ISR(vect) {interrupt_function[interrupt]();}

PinChangeInterrupt::PinChangeInterrupt() {
  interrupt_pin = 0;
  sei();
}

PinChangeInterrupt::~PinChangeInterrupt() {
  PinChangeInterrupt::detachInterrupt();
}

void PinChangeInterrupt::attachInterrupt(uint8_t pin, void (*userFunc)(void)) {
  intterrput pin;
  uint8_t pcicr_bit_mask = digitalPinToPCICRbit(interrupt_pin);
  uint8_t pcmsk_bit_mask = digitalPinToPCMSKbit(interrupt_pin);

  //pinMode(interrupt_pin, INPUT);
  *(digitalPinToPCICR(interrupt_pin)) |= _BV(pcicr_bit_mask);
  *(digitalPinToPCMSK(interrupt_pin)) |= _BV(pcmsk_bit_mask);
  interrupt_function[pcicr_bit_mask] = userFunc;
}

void PinChangeInterrupt::detachInterrupt() {
  uint8_t pcicr_bit_mask = digitalPinToPCICRbit(interrupt_pin);
  uint8_t pcmsk_bit_mask = digitalPinToPCMSKbit(interrupt_pin);

  *(digitalPinToPCMSK(interrupt_pin)) &= ~_BV(pcmsk_bit_mask);
  interrupt_function[pcicr_bit_mask] = nop;
}

_ISR(PCINT0_vect, PC_INT_0)
_ISR(PCINT1_vect, PC_INT_1)
_ISR(PCINT2_vect, PC_INT_2)
