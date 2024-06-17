// 2024_04_29 
// Send 't1' from the computer to the microprocessor. At that, perform a temperature measurement and return a value.

#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

#include <xc.h>
#include <string.h>     // Include this library for strcmp function
#include "interrupts.h"
#include "peripherals.h"

uint16_t instr; //Last character received from the computer
uint8_t measurement_quantity = 100; //Número de mediciones hechas en cada modo
uint16_t voltage_measurements[] = {
    0, 1744, 3654, 5495, 7336, 9132, 10973, 12769, 14610, 16406, 18225, 20044, 21840, 23636, 25432, 27228, 29002, 30798, 32572, 34346, 36097, 37871, 39577, 41283, 42967, 44561, 46066, 47458, 48715, 49860, 50803, 51589, 52217, 52734, 53138, 53475, 53677, 53879, 53991, 54126, 54216, 54261, 54305, 54350, 54373, 54395, 54418, 54440, 54440, 54463, 54463, 54463, 54463, 54463, 54463, 54463, 54508, 54485, 54508, 54508, 54485, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54508, 54543
};

uint16_t current_measurements[] = {
    3580, 3580, 3570, 3570, 3582, 3570, 3570, 3570, 3570, 3570, 3557, 3557, 3557, 3557, 3545, 3545, 3533, 3533, 3521, 3508, 3484, 3459, 3410, 3361, 3275, 3140, 2969, 2735, 2453, 2147, 1828, 1521, 1239, 993, 785, 613, 478, 380, 294, 233, 184, 147, 123, 98, 73, 61, 49, 49, 37, 24, 24, 24, 12, 12, 12, 12, 12, 12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0    
};

void main() 
{
    LED_init();
    
    //Necesitamos configurar los pines de control de parte de potencia.
    power_controller_init();

    //We need  to set up PBCLK2, as both the UART and the SPI modules use it.
    PB2_init(); // PBCLK2=SYSCLK/2=8MHz/2=4MHz
    
    // We now set up the RS485 protocol to communicate with the computer.
    RS485_init();
    // Used configuration: 9600 bauds/s, 8 data bits, no parity, 1 stop bit
    // To read last received instruction: uint16_t = RS485_receive();
    // To send a byte to the computer use: RS485_send1by(s); 
    
    SPI_init();
    
    ADC_init();
    
    __builtin_enable_interrupts();
    
    while(1)
    {
      
        if (instr=='hi') {
            RS485_send2by('hi');
            instr=0;
        }
        if (instr=='v1') {  //Retornar voltaje medido por voltímetro #1
            uint16_t vm1_config = get_vm1_config(); //Cuatro modos de operación: 50V , 100V , 500V y 1500V
            RS485_send2by(vm1_config);
            instr=0;
        }
        
        if (instr=='c0') {      //La capacitancia se elige automáticamente
            LATEbits.LATE0=0;
            LATEbits.LATE1=0;
            LATEbits.LATE2=0;
            RS485_send2by('c0');    // Se envía c0 de vuelta para confirmar la transacción 
            instr=0;
        }
        
        if (instr=='c1') {
            LATEbits.LATE0=1;
            LATEbits.LATE1=0;
            LATEbits.LATE2=0;
            
            RS485_send2by('c1');
            instr=0;
        }
        
        if (instr=='c2') {
            LATEbits.LATE0=0;
            LATEbits.LATE1=1;
            LATEbits.LATE2=0;
            RS485_send2by('c2');
            instr=0;
        }
        
        if (instr=='c3') {
            LATEbits.LATE0=0;
            LATEbits.LATE1=0;
            LATEbits.LATE2=1;
            RS485_send2by('c3');
            instr=0; 
        }
        
        if (instr=='ov') { //Read power card overheat indicators
            uint8_t overheat_indicators = 0; //PORTAbits.RA1+PORTAbits.RA2+PORTAbits.RA3+PORTAbits.RA4+PORTAbits.RA6+PORTAbits.RA7+PORTAbits.RA14+PORTAbits.RA15
            overheat_indicators |= PORTGbits.RG0 << 1;   // Shift RG0 (RAD45) to 0th bit position and OR it with byte
            overheat_indicators |= PORTGbits.RG1 << 0;   // Shift RG1 (RAD70) to 1st bit position and OR it with byte
            RS485_send1by(overheat_indicators);
            instr=0;
        }
        
        if (instr=='t1') {  //t1= temperature sensor 1
            uint16_t t1 = Pt100_1_measurement();
            RS485_send2by(t1);
            instr=0; // Clear instruction
        }
        
        if (instr=='t2') {  //t2= temperature sensor 2
            uint16_t t2 = Pt100_2_measurement();
            RS485_send2by(t2);
            instr=0; // Clear instruction
        }
        
        if (instr=='t3') {  //t3= temperature sensor 3 (RAD temperature sensor)
            uint16_t t3 = Pt100_RAD_measurement();
            RS485_send2by(t3);
            instr=0; // Clear instruction
        }
        
        if (instr=='t4') {  //t4= temperature sensor 4 (module #1's front side)
            uint16_t t4 = Pt100_NES1F_measurement();
            RS485_send2by(t4);
            instr=0; // Clear instruction
        }
        
        if (instr=='t5') {  //t5= temperature sensor 5 (module #1's rear side)
            uint16_t t5 = Pt100_NES1T_measurement();
            RS485_send2by(t5);
            instr=0; // Clear instruction
        }
        
        if (instr=='t6') {  //t6= temperature sensor 6 (module #2's front side)
            uint16_t t6 = Pt100_NES2F_measurement();
            RS485_send2by(t6);
            instr=0; // Clear instruction
        }
        
        if (instr=='t7') {  //t7= temperature sensor  (module #2's rear side)
            uint16_t t7 = Pt100_NES2T_measurement();
            RS485_send2by(t7);
            instr=0; // Clear instruction
        }
        
        if (instr=='t8') {  //t8= temperature sensor 8 (extra temperature sensor)
            uint16_t t8 = Pt100_NES2F_measurement();
            RS485_send2by(t8);
            instr=0; // Clear instruction
        }
        
        if (instr=='t9') {  //t9= temperature sensor 9 (WPVS temperature sensor)
            uint16_t t9 = Pt100_NES2T_measurement();
            RS485_send2by(t9);
            instr=0; // Clear instruction
        }
        
        if (instr=='l1') {  //l1 = light sensor 1 (Front side of pv module #1)
            uint16_t l1 = light_1_measurement();
            RS485_send2by(l1);
            instr=0; // Clear instruction
        }
        
        if (instr=='l2') {  //l2 = light sensor 2 (Rear side of pv module #1)
            uint16_t l2 = light_2_measurement();
            RS485_send2by(l2);
            instr=0; // Clear instruction
        }
        
        if (instr=='l3') {  //l3 = light sensor 3 (Front side of pv module #2)
            uint16_t l3 = light_3_measurement();
            RS485_send2by(l3);
            instr=0; // Clear instruction
        }
        
        if (instr=='l4') {  //l4 = light sensor 4 (Rear side of pv module #2)
            uint16_t l4 = light_4_measurement();
            RS485_send2by(l4);
            instr=0; // Clear instruction
        }
        
        if (instr=='l5') {  //l5 = light sensor 5 (Extra light sensor)
            uint16_t l5 = light_5_measurement();
            RS485_send2by(l5);
            instr=0; // Clear instruction
        }
        
        if (instr=='l6') {  //l6 = light sensor 6 (WPVS light sensor)
            uint16_t l6 = light_6_measurement();
            RS485_send2by(l6);
            instr=0; // Clear instruction
        }
        
        if (instr=='iv') {
            //Configure RD10 to send data:
            U2STAbits.URXEN = 0; // Disable the U2RX pin
            LATDbits.LATD10 = 1;
            
            RS485_send2by(100); // Later, replace '100' with: get_vm1_config();
            for (int i=0; measurement_quantity>=i; i++) { //sizeof() gives number of bytes
                RS485_send2by(voltage_measurements[i]);
                RS485_send2by(current_measurements[i]);
            }
            LATDbits.LATD10 = 0; // Receive mode
            U2STAbits.URXEN = 1; // Enable the U2RX pin
            instr=0; // Clear instruction
        }

    }

}

