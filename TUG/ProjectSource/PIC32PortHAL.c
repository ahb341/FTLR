
#include <xc.h>
#include <stdbool.h>
#include "PIC32PortHAL.h"
#include "bitdefs.h"
// PIC32MX170F256B Configuration Bit Settings for ME218 operating at 40MHz from internal fast oscillator (8Mhz)

// 'C' source line config statements

// DEVCFG3
//#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
//#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
//#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
//
//// DEVCFG2
//#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
//#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
//#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)
//
//// DEVCFG1
//#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
//#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
//#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
//#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
//#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
//#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
//#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
//#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
//#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
//#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
//#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)
//
//// DEVCFG0
//#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
//#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
//#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
//#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
//#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// All possible pins
const uint32_t AllPins = _Pin_0 | _Pin_1 | _Pin_2 | _Pin_3 | _Pin_4 | _Pin_5 | 
_Pin_6 | _Pin_7 | _Pin_8 | _Pin_9 | _Pin_10 | _Pin_11 | _Pin_12 | _Pin_13 | 
_Pin_14 | _Pin_15;

// Legal IO pins for Ports A & B
const uint32_t IO_Port_A = _Pin_0 | _Pin_1 | _Pin_2 | _Pin_3 | _Pin_4;
const uint32_t IO_Port_B = AllPins;

// Analog pins for Ports A & B
const uint32_t AIO_Port_A = _Pin_0 | _Pin_1;
const uint32_t AIO_Port_B = _Pin_0 | _Pin_1 | _Pin_2 | _Pin_3 | _Pin_12 | 
_Pin_13 | _Pin_14 | _Pin_15;

//	Function Prototypes
int main(void);


/****************************************************************************
Configures the specified pin(s) on the specified port as digital outputs.
****************************************************************************/
bool PortSetup_ConfigureDigitalOutputs(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A and if pins passed are legal IO
    if ((WhichPort == _Port_A) && ((WhichPin & IO_Port_A) == WhichPin) && (WhichPin != 0)) {
        
        // Disable pull-up/pull-down resistors
        CNPDACLR = WhichPin;
        CNPUBCLR = WhichPin;
        
        if (WhichPin & AIO_Port_A) {    // Check if pins are analog capable and set to digital
            ANSELACLR = (WhichPin & AIO_Port_A);
        }
        
        TRISACLR = WhichPin;    // Set as output
        
        ReturnVal = true;   // Change return value to true
    }
    
    // Same procedure for Port B
    else if ((WhichPort == _Port_B) && ((WhichPin & IO_Port_B) == WhichPin) && (WhichPin != 0)) {
        
        CNPDBCLR = WhichPin;
        CNPUBCLR = WhichPin;
        
        if (WhichPin & AIO_Port_B) {
            ANSELBCLR = (WhichPin & AIO_Port_B);
        }
        
        TRISBCLR = WhichPin;
        
        ReturnVal = true;
    }
    
    return ReturnVal;
}


/****************************************************************************
Configures the specified pin(s) on the specified port as digital inputs, 
disabling analog input(s). 
****************************************************************************/
bool PortSetup_ConfigureDigitalInputs(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A and if pins passed are legal IO
    if ((WhichPort == _Port_A) &&((WhichPin & IO_Port_A) == WhichPin) && (WhichPin != 0)) {
        
        // Check if pins passed are analog capable and set to digital if so
        if (WhichPin & AIO_Port_A) {
            ANSELACLR = (WhichPin & AIO_Port_A);
        }
        
        TRISASET = WhichPin;    // Set as output
        
        ReturnVal = true;   // Change return value to true
    }
    
    // Same procedure for Port B
    else if ((WhichPort == _Port_B) && ((WhichPin & IO_Port_B) == WhichPin) && (WhichPin != 0)) {
        
        if (WhichPin & AIO_Port_B) {
            ANSELBCLR = (WhichPin & AIO_Port_B);
        }
        
        TRISBSET = WhichPin;
        
        ReturnVal = true;
    }
    
    return ReturnVal;
}


/****************************************************************************
Configures the specified pin(s) on the specified port as analog inputs.
****************************************************************************/
bool PortSetup_ConfigureAnalogInputs(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A and if pins passed are legal analog pins
    if ((WhichPort == _Port_A) && ((WhichPin & AIO_Port_A) == WhichPin) && (WhichPin != 0)) {
        
        ANSELASET = (WhichPin & AIO_Port_A);    // Enable analog
        TRISASET = WhichPin;    // Set as input
        
        ReturnVal = true;   // Change return value to true
    }
    
    // Same procedure for Port B
    else if ((WhichPort == _Port_B) && ((WhichPin & AIO_Port_B) == WhichPin) && (WhichPin != 0)) {
        
        ANSELBSET = (WhichPin & AIO_Port_B);
        TRISBSET = WhichPin;
        
        ReturnVal = true;
    }
    
    return ReturnVal;
}


/****************************************************************************
Configures the specified pin(s) on the specified port with weak pull-ups.
****************************************************************************/
bool PortSetup_ConfigurePullUps(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin){
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A and if pins passed are legal
    if ((WhichPort == _Port_A) && ((WhichPin & IO_Port_A) == WhichPin) && (WhichPin != 0)) {
        
        TRISASET = WhichPin;    // Ensure pins are set to input
        CNPDACLR = WhichPin;    // Ensure pull-downs are disabled
        CNPUASET = WhichPin;    // Enable pull-ups
        
        ReturnVal = true;   // Change return value to true
    }
    
    // Same procedure for Port B
    else if ((WhichPort == _Port_B) && ((WhichPin & IO_Port_B) == WhichPin) && (WhichPin != 0)) {
        
        TRISBSET = WhichPin;
        CNPDBCLR = WhichPin;
        CNPUBSET = WhichPin;
        
        ReturnVal = true;
    }
    
    return ReturnVal;
}


/****************************************************************************
Configures the specified pin(s) on the specified port with weak pull-downs.
****************************************************************************/
bool PortSetup_ConfigurePullDowns(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A and if pins passed are legal
    if ((WhichPort == _Port_A) && ((WhichPin & IO_Port_A) == WhichPin) && (WhichPin != 0)) {
        
        CNPUACLR = WhichPin;    // Ensure pull-ups are disabled
        CNPDASET = WhichPin;    // Enable pull-downs
        
        ReturnVal = true;   // Change return value to true
        
    }
    
    // Same procedure for Port B
    else if ((WhichPort == _Port_B) && ((WhichPin & IO_Port_B) == WhichPin) && (WhichPin != 0)) {
        
        CNPUBCLR = WhichPin;
        CNPDBSET = WhichPin;
        
        ReturnVal = true;
    }
    return ReturnVal;
}


/****************************************************************************
Configures the specified pin(s) on the specified port as open-drain outputs.
****************************************************************************/
bool PortSetup_ConfigureOpenDrain(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin){
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A and if pins passed are legal
    if ((WhichPort == _Port_A) && ((WhichPin & IO_Port_A) == WhichPin) && (WhichPin != 0)) {
        
        TRISACLR = WhichPin;    // Ensure pins are set as output
        ODCASET = WhichPin;     // Enable open-drain
        
        ReturnVal = true;   // Change return value to true
    }
    
    // Same procedure for Port B
    else if ((WhichPort == _Port_B) && ((WhichPin & IO_Port_B) == WhichPin) && (WhichPin != 0)) {
        
        TRISBCLR = WhichPin;
        ODCBSET = WhichPin;
        
        ReturnVal = true;
    }
    
    return ReturnVal;
}


/****************************************************************************
Configures the specified pin(s) on the specified port to enable change 
notifications. If any bits are set in the PortSetup_Pin_t parameter, then 
change notifications are enabled. If that parameter is 0, then change 
notifications are disabled globally.
****************************************************************************/
bool PortSetup_ConfigureChangeNotification(PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin){
    
    bool ReturnVal = false;     // Initialize return value as false
    
    // Check if Port A
    if (WhichPort == _Port_A) {
        
        // If 0 passed in as pin, disable CN on all of Port A
        if (WhichPin == 0) {
            CNCONACLR = BIT15HI;
        }
        
        // If pin(s) are legal
        else if ((WhichPin & IO_Port_A) == WhichPin){
            
            CNCONASET = BIT15HI;    // Enable CN on Port A
            CNENASET = WhichPin;    // Enable CN on pin(s)
            
            ReturnVal = true;   // Change return value to true
        }
    }
    
    // Same procedure for Port B
    else if (WhichPort == _Port_B) {
        
        if (WhichPin == 0) {
            CNCONBCLR = BIT15HI;
        }
        
        else if ((WhichPin & IO_Port_B) == WhichPin) {
            
            CNCONBSET = BIT15HI;
            CNENBSET = WhichPin;
            
            ReturnVal = true;
        }
    }
    
    return ReturnVal;
}
