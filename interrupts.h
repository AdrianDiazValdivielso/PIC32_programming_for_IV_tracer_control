
#ifndef INTERRUPTS_H    /* Guard against multiple inclusion */
#define INTERRUPTS_H

// Declare external variable for received_character
extern uint16_t instr; //Here we store the last instruction received from the computer

// Function prototype declaration
void U2Interrupt(void);

#endif /* INTERRUPTS_H */

