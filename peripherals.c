
#include <xc.h>
#include "peripherals.h"

void LED_init()
{
    // Configuración de pin de diodo LED
    TRISKbits.TRISK5 = 0; // Set RK5 (LED) as an output
    LATKbits.LATK5 = 0; // By changing this bit we can control the LED. '1' means ON, while '0' means 0FF.    
}

void PB2_init() //Set up peripheral bus 2 clock
{
    asm volatile("di"); // Disable all interrupts (Alternatively: "__builtin_disable_interrupts();")
    SYSKEY = 0xAA996655; //Start unlock sequence
    SYSKEY = 0x556699AA; //Second part of the unlock sequence. Now we can modify the PB2DIV register.
    // PB2DIV 
    PB2DIVbits.ON = 1; // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB2DIVbits.PBDIV = 1; // Peripheral Bus 2 Clock Divisor Control (PBCLK2 is SYSCLK divided by 2, that is, 8MHz/2=4MHz)
    // Lock Sequence
    SYSKEY = 0x33333333; //Lock again by writing a random number
    asm volatile("ei"); // Enable all interrupts
}

void RS485_init() // We use UART1 to transmit and UART2 to receive
{
    //UART1 configuration (transmission)
    RPD11Rbits.RPD11R=0b0001; //Configure RPD11 as UART1's T pin.     
    TRISDbits.TRISD11 = 0; // Configure RD1 as output (Tx)
    
    //UART2 configuration (reception)
    U2RXR = 0b0000; //Configure RPD9 as UART2's R pin.
    TRISDbits.TRISD9 = 1; // Configure RD2 as input (Rx)
    
    //Configure pin RD10 as output R/T
    TRISDbits.TRISD10 = 0; // Configure RD1 as output (R/T)    
    
    //UART1 initialization
    U1MODE = 0; // Set UART 1 off prior to setting it up
    U1MODEbits.BRGH = 0; //Standard Speed mode - 16x baud clock enabled (not necessary, as we already set the whole u1mode register to 0)
    U1BRG = 4000000 / (16 * 9600) - 1;// This is the formula straight from the datasheet (PBCLK2 frequency is 4MHz)
    U1STA = 0; // Disable the TX and RX pins, clear all flags
    U1STAbits.UTXEN = 1; // Enable the TX pin
    U1MODEbits.PDSEL = 0; //8-bit data, no parity
    U1MODEbits.STSEL = 0; //1 stop bit
    U1MODEbits.ON = 1; // Turn on the UART 1 peripheral
    
    //UART2 initialization
    U2MODE = 0; // Set UART 2 off prior to setting it up
    U2MODEbits.BRGH = 0; //Standard Speed mode - 16x baud clock enabled (not necessary, as we already set the whole u1mode register to 0)
    U2BRG = 4000000 / (16 * 9600) - 1;// This is the formula straight from the datasheet (PBCLK2 frequency is 4MHz)
    U2STA = 0; // Disable the TX and RX pins, clear all flags
    U2STAbits.URXEN = 1; // Enable the RX pin
    U2MODEbits.PDSEL = 0b00; //8-bit data, no parity
    U2MODEbits.STSEL = 0; //1 stop bit
    U2MODEbits.ON = 1; // Turn on the UART 2 peripheral
    
    //Interrupt configuration (we want interrupts for reception)
    IPC36bits.U2RXIP = 4; //Set UART2 receive done priority to 4 
    IEC4bits.U2RXIE = 1;  //Enable interrupt for receive done
    U2STAbits.URXISEL = 0b01; //Interrupt is asserted while receive buffer is 1/2 or more full
}

uint16_t RS485_receive()
{
    return instr;
}

void RS485_send1by(char s) //send one byte
{
    
    //Configure RD10 to send data:
    U2STAbits.URXEN = 0; // Disable the U2RX pin
    LATDbits.LATD10 = 1; // Transmit mode
    U1TXREG = s; // Put the character we want to send in UART2's transmit register
    while (U1STAbits.TRMT==0); // Wait until transmission is complete
    LATDbits.LATD10 = 0; // Receive mode
    U2STAbits.URXEN = 1; // Enable the U2RX pin
}

void RS485_send2by(uint16_t s) //send two bytes
{
    //Configure RD10 to send data:
    U2STAbits.URXEN = 0; // Disable the U2RX pin
    LATDbits.LATD10 = 1; // Send mode
    U1TXREG = s>>8; // Send the most significant byte first
    U1TXREG = s; // Send the least significant byte later
    while (U1STAbits.TRMT==0); // Wait until transmission is complete
    LATDbits.LATD10 = 0; // Receive mode
    U2STAbits.URXEN = 1; // Enable the U2RX pin
}


/////////////////////// SPI ///////////////////////
void SPI_init()
{
    // Set up PPS (peripheral pin select) for SPI1 on RD2 and RD3
    SDI1R=0b0000;       // Configuring RD2 pin as SDI1 (input)
    RPD3R=0b0101;       // Configuring RD3 pin as SDO1 (output)
    
    // Configure SDI pin as input
    TRISDbits.TRISD2 = 1; // Configure RD2 as input (SDI)
    
    // Configure SCK and SDO pins as outputs
    TRISDbits.TRISD3 = 0; // Configure RD3 as output (SDO)
    TRISDbits.TRISD1 = 0; // Configure RD1 as output (SCK1)
    
    // For each sensor, configure its chip Select pin as output, set it to '1' and configure the Data Ready pin as input
    // Temperature sensor Pt100_1
    TRISHbits.TRISH9 = 0; // Configure RH9 as output (CS_Pt100_1)
    LATHbits.LATH9 = 1; //Disable temperature sensor Pt100_1
    TRISJbits.TRISJ5 = 1; // Configure RJ5 as input (DRDY1)
    
    // Temperature sensor Pt100_2
    TRISHbits.TRISH10 = 0; // Configure RH10 as output (CS_Pt100_2)
    LATHbits.LATH10 = 1; //Disable temperature sensor Pt100_2
    TRISJbits.TRISJ6 = 1; // Configure RJ6 as input (DRDY2)
    
    // Temperature sensor Pt100_RAD
    TRISHbits.TRISH12 = 0; // Configure RH12 as output (CS_Pt100_RAD)
    LATHbits.LATH12 = 1; //Disable temperature sensor Pt100_RAD
    TRISJbits.TRISJ8 = 1; // Configure RJ8 as input (DRDYRAD)
    
    // Temperature sensor Pt100_NES1F
    TRISHbits.TRISH5 = 0; // Configure RH5 as output (CS_Pt100_NES1F)
    LATHbits.LATH5 = 1; //Disable temperature sensor Pt100_NES1F
    TRISJbits.TRISJ1 = 1; // Configure RJ1 as input (DRDY1F)
    
    // Temperature sensor Pt100_NES1T
    TRISHbits.TRISH4 = 0; // Configure RH4 as output (CS_Pt100_NES1T)
    LATHbits.LATH4 = 1; //Disable temperature sensor Pt100_NES1T
    TRISJbits.TRISJ0 = 1; // Configure RJ0 as input (DRDY1T)
    
    // Temperature sensor Pt100_NES2F
    TRISHbits.TRISH7 = 0; // Configure RH7 as output (CS_Pt100_NES2F)
    LATHbits.LATH7 = 1; //Disable temperature sensor Pt100_NES2F
    TRISJbits.TRISJ3 = 1; // Configure RJ3 as input (DRDY2F)
    
    // Temperature sensor Pt100_NES2T
    TRISHbits.TRISH6 = 0; // Configure RH6 as output (CS_Pt100_NES2T)
    LATHbits.LATH6 = 1; //Disable temperature sensor Pt100_NES2T
    TRISJbits.TRISJ2 = 1; // Configure RJ2 as input (DRDY2T)
    
    // Temperature sensor Pt100_NES_EX
    TRISHbits.TRISH8 = 0; // Configure RH8 as output (CS_Pt100_NES_EX)
    LATHbits.LATH8 = 1; //Disable temperature sensor Pt100_NES_EX
    TRISJbits.TRISJ4 = 1; // Configure RJ4 as input (DRDYEX)
    
    // Temperature sensor Pt100_WPVS
    TRISHbits.TRISH11 = 0; // Configure RH11 as output (CS_Pt100_WPVS)
    LATHbits.LATH11 = 1; //Disable temperature sensor Pt100_WPVS
    TRISJbits.TRISJ7 = 1; // Configure RJ7 as input (DRDYWPVS)
    /*IMPORTANT: For each new sensor added, modify the t_sensor_init function too!*/
    
    // SPI module configuration
    SPI1CONbits.ON = 0; // Turn off SPI1 before configuring
    SPI1CONbits.MSTEN = 1; // Enable Master mode
    SPI1CONbits.CKP = 0; // Clock signal is active low, idle state is high
    SPI1CONbits.CKE = 0; // Data is shifted out/in on transition from idle (high) state to active (low) state
    SPI1CONbits.SMP = 0; // Input data is sampled at the end of the clock signal
    SPI1CONbits.MODE16 = 0; // Do not use 16-bit mode
    SPI1CONbits.MODE32 = 0; // Do not use 32-bit mode (together with the previous, 8-bit mode is set)
    SPI1BRG = 0; // Baud Rate = F_PB/(2(0+1))=2MHz, where F_PB=4MHz.
    SPI1CONbits.ENHBUF = 0; // Disables Enhanced Buffer mode
    SPI1CONbits.ON = 1; // Configuration is done, turn on SPI1 peripheral
}

uint8_t SPI_send(char data)
{
    SPI1BUF = data; // Send **data** to SPI1BUF.
    while (SPI1STATbits.SPIRBF==0); // While the receive buffer is empty, loop
    return (uint8_t)SPI1BUF; // Return whatever is in SPI1BUF
}

uint16_t Pt100_1_measurement() //Initializes temperature sensors and starts a measurement
{
    //We enable the peripheral and configure it to perform a temperature measurement
    LATHbits.LATH9 = 0; // Enable temperature sensor Pt100_1
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH9 = 1; // Disable temperature sensor Pt100_1 (This tells the sensor communication has ended)
    
    //Wait for temperature measurement to be ready.
    delay_us(75000);   
    //According to the datasheet, this takes at most 66ms)
    
    //Another alternative is polling the conversion ready pin: "while(PORTJbits.RJ5==1);" 
    //This is faster, but the microprocessor can get stuck waiting if 
    //there is no response (which can happen when the sensor is not installed)
    
    //We ask the peripheral to send the temperature measurement to the microprocessor
    LATHbits.LATH9 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH9 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH9 = 0; // Enable temperature sensor Pt100_1
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH9 = 1; // Disable temperature sensor Pt100_1 (This tells the sensor communication has ended)
    
    return rtd>>1;
}    

uint16_t Pt100_2_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH10 = 0; // Enable temperature sensor Pt100_2
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH10 = 1; // Disable temperature sensor Pt100_2 (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH10 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH10 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH10 = 0; // Enable temperature sensor Pt100_2
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH10 = 1; // Disable temperature sensor Pt100_2 (This tells the sensor communication has ended)
    
    return rtd>>1;
}    

uint16_t Pt100_RAD_measurement() //Initializes temperature sensors and starts a measurement
{
    //We enable the peripheral and configure it to perform a temperature measurement
    LATHbits.LATH12 = 0; // Enable temperature sensor Pt100_RAD
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH12 = 1; // Disable temperature sensor Pt100_RAD (This tells the sensor communication has ended)
 
    delay_us(75000);   //Wait for temperature measurement to be ready 
    //According to the datasheet, this takes at most 66ms)
    
    //We ask the peripheral to send the temperature measurement to the microprocessor
    LATHbits.LATH12 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH12 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH12 = 0; // Enable temperature sensor Pt100_RAD
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH12 = 1; // Disable temperature sensor Pt100_RAD (This tells the sensor communication has ended)
    
    return rtd>>1;
}    

uint16_t Pt100_NES1F_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH5 = 0; // Enable temperature sensor Pt100_NES1F
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH5 = 1; // Disable temperature sensor Pt100_NES1F (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH5 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH5 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH5 = 0; // Enable temperature sensor Pt100_NES1F
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH5 = 1; // Disable temperature sensor Pt100_NES1F (This tells the sensor communication has ended)
    return rtd>>1;
}    
   
uint16_t Pt100_NES1T_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH4 = 0; // Enable temperature sensor Pt100_NES1T
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH4 = 1; // Disable temperature sensor Pt100_NES1T (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH4 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH4 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH4 = 0; // Enable temperature sensor Pt100_NES1T
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH4 = 1; // Disable temperature sensor Pt100_NES1T (This tells the sensor communication has ended)
    return rtd>>1;
}

uint16_t Pt100_NES2F_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH7 = 0; // Enable temperature sensor Pt100_NES2F
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH7 = 1; // Disable temperature sensor Pt100_NES2F (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH7 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH7 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH7 = 0; // Enable temperature sensor Pt100_NES2F
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH7 = 1; // Disable temperature sensor Pt100_NES2F (This tells the sensor communication has ended)
    return rtd>>1;
}

uint16_t Pt100_NES2T_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH6 = 0; // Enable temperature sensor Pt100_NES2F
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH6 = 1; // Disable temperature sensor Pt100_NES2F (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH6 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH6 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH6 = 0; // Enable temperature sensor Pt100_NES2T
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH6 = 1; // Disable temperature sensor Pt100_NES2T (This tells the sensor communication has ended)
    return rtd>>1;
}

uint16_t Pt100_EX_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH8 = 0; // Enable temperature sensor Pt100_NES2F
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH8 = 1; // Disable temperature sensor Pt100_NES2F (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH8 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH8 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH8 = 0; // Enable temperature sensor Pt100_EX
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH8 = 1; // Disable temperature sensor Pt100_EX (This tells the sensor communication has ended)
    return rtd>>1;
}

uint16_t Pt100_WPVS_measurement() //Initializes temperature sensors and starts a measurement
{
    LATHbits.LATH11 = 0; // Enable temperature sensor Pt100_WPVS
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0xA3); // Set bias voltage. Prepare one temperature conversion. Clear fault bits (optional). Configure 50Hz reject mode.
    LATHbits.LATH11 = 1; // Disable temperature sensor Pt100_WPVS (This tells the sensor communication has ended)
    
    delay_us(75000);   //Wait for temperature measurement to be ready (this usually takes around 62.5ms)
    
    LATHbits.LATH11 = 0; 
    SPI_send(0x01); // We want to read the chip's 0x01 and 0x02 registers, where the measurement is stored
    uint16_t rtd = SPI_send(0x00)<< 8; // Read register 0x01 (most significant byte)
    rtd |= SPI_send(0x00); // Read register 0x02 (least significant byte)
    LATHbits.LATH11 = 1; // End communication
    
    //We disable the bias voltage to prevent overheating
    LATHbits.LATH11 = 0; // Enable temperature sensor Pt100_WPVS
    SPI_send(0x80); // This tells the sensor we want to write to configure register
    SPI_send(0x03); // Disable bias voltage to prevent overheating
    LATHbits.LATH11 = 1; // Disable temperature sensor Pt100_WPVS (This tells the sensor communication has ended)
    return rtd>>1;
}
    /////////////////////////// LIGHT SENSORS ///////////////////////////
void ADC_init()
{
    /* Step 1 - Configure the analog port pins*/
    TRISBbits.TRISB5 = 1; // Configure RB5 as input (not necessary, as this is the default)
    ANSELBbits.ANSB5 = 1; // Configure RB5 as analog pin (not necessary, as this is the default)
    
    /*Step 2 -  Initialize the ADC calibration values*/
    ADC0CFG = DEVADC0;
    ADC1CFG = DEVADC1;
    ADC2CFG = DEVADC2;
    ADC3CFG = DEVADC3;
    ADC4CFG = DEVADC4;
    
    /*Step 3 - Select the analog inputs to the ADC multiplexers*/
    ADCTRGMODEbits.SH0ALT = 0b01; //ADC0=AN45 selected
    ADCTRGMODEbits.SH1ALT = 0b01; //ADC1=AN46 selected
    ADCTRGMODEbits.SH2ALT = 0b01; //ADC2=AN47 selected
    ADCTRGMODEbits.SH3ALT = 0b01; //ADC3=AN48 selected
    ADCTRGMODEbits.SH4ALT = 0b01; //ADC4=AN49 selected

    
    /*Step 4 -  Select the format of the ADC result*/ //I think it is not necessary
    ADCIMCON1bits.DIFF0 = 0; // AN0/AN45 Mode bit (AN45 is using Single-ended mode)
    ADCIMCON1bits.SIGN0 = 0; // AN0/AN45 Signed Data Mode bit (AN45 is using Unsigned Data mode)
    ADCIMCON1bits.DIFF1 = 0; // AN1/AN46 Mode bit (AN46 is using Single-ended mode)
    ADCIMCON1bits.SIGN1 = 0; // AN1/AN46 Signed Data Mode bit (AN46 is using Unsigned Data mode)
    ADCIMCON1bits.DIFF2 = 0; // AN2/AN47 Mode bit (AN47 is using Single-ended mode)
    ADCIMCON1bits.SIGN2 = 0; // AN2/AN47 Signed Data Mode bit (AN47 is using Unsigned Data mode)
    ADCIMCON1bits.DIFF3 = 0; // AN3/AN48 Mode bit (AN48 is using Single-ended mode)
    ADCIMCON1bits.SIGN3 = 0; // AN3/AN48 Signed Data Mode bit (AN48 is using Unsigned Data mode)
    ADCIMCON1bits.DIFF4 = 0; // AN4/AN49 Mode bit (AN4/AN49 is using Single-ended mode)
    ADCIMCON1bits.SIGN4 = 0; // AN4/AN49 Signed Data Mode bit (AN4/AN49 is using Unsigned Data mode)
    
    /*Step 5 - Select the conversion trigger source*/
    ADCTRG1bits.TRGSRC0 = 1; //Trigger Source for Conversion of Analog Input AN0/AN45 Select bits (GSWTRG)
    ADCTRG1bits.TRGSRC1 = 1; //Trigger Source for Conversion of Analog Input AN1/AN46 Select bits (GSWTRG)
    ADCTRG1bits.TRGSRC2 = 1; //Trigger Source for Conversion of Analog Input AN2/AN47 Select bits (GSWTRG)
    ADCTRG1bits.TRGSRC3 = 1; //Trigger Source for Conversion of Analog Input AN3/AN48 Select bits (GSWTRG)
    ADCTRG2bits.TRGSRC4 = 1; //Trigger Source for Conversion of Analog Input AN4/AN49 Select bits (GSWTRG)

    //I may need to change this
    
    /*Step 6 - Select the voltage reference source*/
    ADCCON3bits.VREFSEL = 0; // Select AVDD and AVSS as reference source
    
    /*Step 7 - Select the scanned inputs*/
    ADCCSS1bits.CSS0 = 1; // Select AN0/AN45 for input scan
    ADCCSS1bits.CSS1 = 1; // Select AN1/AN46 for input scan
    ADCCSS1bits.CSS2 = 1; // Select AN2/AN47 for input scan
    ADCCSS1bits.CSS3 = 1; // Select AN3/AN48 for input scan
    ADCCSS1bits.CSS4 = 1; // Select AN4/AN49 for input scan

    
    /*Step 8 -  Select the analog-to-digital conversion clock source and prescaler*/
    ADCCON3bits.ADCSEL = 0; // Select input clock source (Internal RC oscillator selected)
    ADCCON3bits.CONCLKDIV = 0; // Control clock frequency is same of input clock
    ADC0TIMEbits.ADCDIV = 1; // ADC0 clock frequency (TAD0) is half of control clock
    ADC1TIMEbits.ADCDIV = 1; // ADC1 clock frequency (TAD1) is half of control clock
    ADC2TIMEbits.ADCDIV = 1; // ADC2 clock frequency (TAD2) is half of control clock
    ADC3TIMEbits.ADCDIV = 1; // ADC3 clock frequency (TAD3) is half of control clock
    ADC4TIMEbits.ADCDIV = 1; // ADC4 clock frequency (TAD4) is half of control clock
    
    
    /*Step 9 - Specify any additional acquisition time, if required*/
    ADC0TIMEbits.SAMC = 5; // ADC0 sampling time = 5 * TAD0
    ADC0TIMEbits.SELRES = 3; // ADC0 resolution is 12 bits
    ADC1TIMEbits.SAMC = 5; // ADC1 sampling time = 5 * TAD0
    ADC1TIMEbits.SELRES = 3; // ADC1 resolution is 12 bits
    ADC2TIMEbits.SAMC = 5; // ADC2 sampling time = 5 * TAD0
    ADC2TIMEbits.SELRES = 3; // ADC2 resolution is 12 bits
    ADC3TIMEbits.SAMC = 5; // ADC3 sampling time = 5 * TAD0
    ADC3TIMEbits.SELRES = 3; // ADC3 resolution is 12 bits
    ADC4TIMEbits.SAMC = 5; // ADC4 sampling time = 5 * TAD0
    ADC4TIMEbits.SELRES = 3; // ADC4 resolution is 12 bits
    
    /*Step 10 - Turn on the ADC module*/
    ADCCON1bits.ON = 1;
    
    /*Step 11 - Poll (or wait for the interrupt) for the voltage reference to be ready*/
    while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT); // Wait if there is a fault with the reference voltage
    
    /*Step 12 -  Enable the analog and bias circuit for required ADC modules and after the ADC module
wakes-up, enable the digital circuit*/
    ADCANCONbits.ANEN0 = 1; // Enable the clock to analog bias ADC0
    ADCANCONbits.ANEN1 = 1; // Enable the clock to analog bias ADC0
    ADCANCONbits.ANEN2 = 1; // Enable the clock to analog bias ADC0
    ADCANCONbits.ANEN3 = 1; // Enable the clock to analog bias ADC0
    ADCANCONbits.ANEN4 = 1; // Enable the clock to analog bias ADC0
    
    /* Wait for ADC to be ready */
    while(!ADCANCONbits.WKRDY0); // Wait until ADC0 is ready
    while(!ADCANCONbits.WKRDY1); // Wait until ADC1 is ready
    while(!ADCANCONbits.WKRDY2); // Wait until ADC2 is ready
    while(!ADCANCONbits.WKRDY3); // Wait until ADC3 is ready
    while(!ADCANCONbits.WKRDY4); // Wait until ADC4 is ready
    
    /* Enable the ADC module */
    ADCCON3bits.DIGEN0 = 1; // Enable ADC0
    ADCCON3bits.DIGEN1 = 1; // Enable ADC1
    ADCCON3bits.DIGEN2 = 1; // Enable ADC2
    ADCCON3bits.DIGEN3 = 1; // Enable ADC3
    ADCCON3bits.DIGEN4 = 1; // Enable ADC4
    
}

uint16_t light_1_measurement()  //AN45 --> ADC0 //PV module 1's front sensor
{   
    /* Clear the result register to avoid reading stale data */
    volatile uint16_t dummy = ADCDATA0;
    /* Trigger a conversion */
    ADCCON3bits.GSWTRG = 1;
    /* Wait the conversions to complete */
    while (ADCDSTAT1bits.ARDY0 == 0);
    /* fetch the result */
    return ADCDATA0;
}

uint16_t light_2_measurement() //AN46--> ADC1 //PV module 1's rear sensor
{
    /* Clear the result register to avoid reading stale data */
    volatile uint16_t dummy = ADCDATA1;
    /* Trigger a conversion */
    ADCCON3bits.GSWTRG = 1;
    /* Wait the conversions to complete */
    while (ADCDSTAT1bits.ARDY1 == 0);
    /* fetch the result */
    return ADCDATA1;
}

uint16_t light_3_measurement() //AN47 --> ADC2 //PV module 2's front sensor
{
    /* Clear the result register to avoid reading stale data */
    volatile uint16_t dummy = ADCDATA2;
    /* Trigger a conversion */
    ADCCON3bits.GSWTRG = 1;
    /* Wait the conversions to complete */
    while (ADCDSTAT1bits.ARDY2 == 0);
    /* fetch the result */
    return ADCDATA2;
}

uint16_t light_4_measurement() //AN48 --> ADC3 //PV module 2's rear sensor
{
    /* Clear the result register to avoid reading stale data */
    volatile uint16_t dummy = ADCDATA3;
    /* Trigger a conversion */
    ADCCON3bits.GSWTRG = 1;
    /* Wait the conversions to complete */
    while (ADCDSTAT1bits.ARDY3 == 0);
    /* fetch the result */
    return ADCDATA3;
}

uint16_t light_5_measurement() //AN49 --> ADC4 //Extra light sensor
{
    /* Clear the result register to avoid reading stale data */
    volatile uint16_t dummy = ADCDATA4;
    ADCTRGMODEbits.SH4ALT = 0b01; //ADC4=AN49 selected (necessary since this ADC is shared)
    /* Trigger a conversion */
    ADCCON3bits.GSWTRG = 1;
    /* Wait the conversions to complete */
    while (ADCDSTAT1bits.ARDY4 == 0);
    /* fetch the result */
    return ADCDATA4;
}

uint16_t light_6_measurement() //AN4 --> ADC4 //WPVS light sensor
{
    /* Clear the result register to avoid reading stale data */
    volatile uint16_t dummy = ADCDATA4;
    ADCTRGMODEbits.SH4ALT = 0b00; //ADC4=AN4 selected (necessary since this ADC is shared)
    /* Trigger a conversion */
    ADCCON3bits.GSWTRG = 1;
    /* Wait the conversions to complete */
    while (ADCDSTAT1bits.ARDY4 == 0);
    /* fetch the result */
    return ADCDATA4;
}

/////////////////////// Power Controller ///////////////////////
void power_controller_init()
{
    // Configuración de V5_1
    ANSELAbits.ANSA1 = 0; // Set RA1 (V5_1) as digital input
    
    // Configuración de pines para control de condensadores
    TRISEbits.TRISE0 = 0; // Set RE0 (CONFIG_C1) as an output
    TRISEbits.TRISE1 = 0; // Set RE1 (CONFIG_C2) as an output
    TRISEbits.TRISE2 = 0; // Set RE2 (CONFIG_C3) as an output

}

uint16_t get_vm1_config()
{
    if (PORTAbits.RA1) {                //V5_1
        return 50;
    }
    else if (PORTAbits.RA2) {           //V450_1
        return 450;
    }
    else if (PORTAbits.RA3) {           //V750_1
        return 750;
    }
    else if (PORTAbits.RA4){            //V1500_1       
        return 1500;
    }
}

/////////////////////// Timer ///////////////////////
void delay_us(unsigned int us)
{
    // Convert microseconds us into how many clock ticks it will take
    us *= 8000000 / 1000000 / 2; // Core Timer updates every 2 ticks (1s=1000000us and clock frequency=8000000MHz)

    _CP0_SET_COUNT(0); // Set Core Timer count to 0
    while (us > _CP0_GET_COUNT()){ // Wait until Core Timer count reaches the number we calculated earlier
    }
}