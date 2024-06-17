
#ifndef _PERIPHERALS_H    /* Guard against multiple inclusion */
#define _PERIPHERALS_H

extern uint16_t instr; //Here we store the last character received from the computer

void LED_init();

void PB2_init();

void RS485_init();
uint16_t RS485_receive();
void RS485_send1by(char s);
void RS485_send2by(uint16_t s);

void SPI_init();
uint16_t Pt100_1_measurement();

void ADC_init();
uint16_t light_1_measurement();

#endif /* _PERIPHERALS_H */