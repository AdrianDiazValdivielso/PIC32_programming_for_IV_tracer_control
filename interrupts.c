
#include <xc.h>
#include <sys/attribs.h>
#include "interrupts.h"

void __ISR(_UART2_RX_VECTOR, IPL4SRS) U2Interrupt() { 
    //We first clear the previous instruction
    instr = 0;
    //Read UART2 reception buffer
    //We receive 4 bytes: instr1 + instr2 + \r + \n
    instr = U2RXREG<<8;
    instr |= U2RXREG;
    char dummy = U2RXREG;
    dummy = U2RXREG;
    IFS4CLR=0x00040000; //Clear interrupt flag
}