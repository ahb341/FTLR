#include "PIC32PortHAL.h"
#include "bitdefs.h"
#include <xc.h>
#include <stdio.h>
#include <stdbool.h>

const int PORT_A_LEGAL = _Pin_0 | _Pin_1 | _Pin_2 | _Pin_3 | _Pin_4;
const int PORT_B_LEGAL = _Pin_0 | _Pin_1 | _Pin_2 | _Pin_3 | _Pin_4
| _Pin_5 | _Pin_6 | _Pin_7 | _Pin_8 | _Pin_9 | _Pin_10 | _Pin_11 | _Pin_12
| _Pin_13 | _Pin_14 | _Pin_15;

// PortHAL function implementations
bool PortSetup_ConfigureDigitalInputs( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELACLR = WhichPin;
            ODCACLR = WhichPin;
            CNPUACLR = WhichPin;
            CNPDACLR = WhichPin;
            CNENACLR = WhichPin;
            
            // Configure output
            TRISASET = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELBCLR = WhichPin;
            ODCBCLR = WhichPin;
            CNPUBCLR = WhichPin;
            CNPDBCLR = WhichPin;
            CNENBCLR = WhichPin;
            
            // Configure output
            TRISBSET = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}

bool PortSetup_ConfigureDigitalOutputs( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELACLR = WhichPin;
            ODCACLR = WhichPin;
            CNPUACLR = WhichPin;
            CNPDACLR = WhichPin;
            CNENACLR = WhichPin;
            
            // Configure output
            TRISACLR = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELBCLR = WhichPin;
            ODCBCLR = WhichPin;
            CNPUBCLR = WhichPin;
            CNPDBCLR = WhichPin;
            CNENBCLR = WhichPin;
            
            // Configure output
            TRISBCLR = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}

bool PortSetup_ConfigureAnalogInputs( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ODCACLR = WhichPin;
            CNPUACLR = WhichPin;
            CNPDACLR = WhichPin;
            CNENACLR = WhichPin;
            
            // Configure input
            ANSELASET = WhichPin;
            TRISASET = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ODCBCLR = WhichPin;
            CNPUBCLR = WhichPin;
            CNPDBCLR = WhichPin;
            CNENBCLR = WhichPin;
            
            // Configure input
            ANSELBSET = WhichPin;
            TRISBSET = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}

bool PortSetup_ConfigurePullUps( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELACLR = WhichPin;
            ODCACLR = WhichPin;
            CNPDACLR = WhichPin;
            CNENACLR = WhichPin;
            
            // Configure input
            TRISASET = WhichPin;
            CNPUASET = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELBCLR = WhichPin;
            ODCBCLR = WhichPin;
            CNPDBCLR = WhichPin;
            CNENBCLR = WhichPin;
            
            // Configure input
            TRISBSET = WhichPin;
            CNPUBSET = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}

bool PortSetup_ConfigurePullDowns( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELACLR = WhichPin;
            ODCACLR = WhichPin;
            CNPUACLR = WhichPin;
            CNENACLR = WhichPin;
            
            // Configure input
            TRISASET = WhichPin;
            CNPDASET = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELBCLR = WhichPin;
            ODCBCLR = WhichPin;
            CNPUBCLR = WhichPin;
            CNENBCLR = WhichPin;
            
            // Configure input
            TRISBSET = WhichPin;
            CNPDBSET = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}

bool PortSetup_ConfigureOpenDrain( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELACLR = WhichPin;
            CNPDACLR = WhichPin;
            CNPUACLR = WhichPin;
            CNENACLR = WhichPin;
            
            // Configure output
            TRISACLR = WhichPin;
            ODCASET = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELBCLR = WhichPin;
            CNPDBCLR = WhichPin;
            CNPUBCLR = WhichPin;
            CNENBCLR = WhichPin;
            
            // Configure output
            TRISBCLR = WhichPin;
            ODCBSET = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}

bool PortSetup_ConfigureChangeNotification( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin) {
    bool ReturnVal = true;
    
    // Check if these ports & pins are legal
    if (WhichPort == _Port_A) {
        // Port A
        if ((WhichPin & PORT_A_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELACLR = WhichPin;
            ODCACLR = WhichPin;
            CNPUACLR = WhichPin;
            CNPDACLR = WhichPin;
            
            // Configure input
            TRISASET = WhichPin;
            CNCONAbits.ON = 1;
            CNENASET = WhichPin;
        }
    } 
    else if (WhichPort == _Port_B) {
        // Port B
        if ((WhichPin & PORT_B_LEGAL) == 0) {
            // Not legal port & pin combination
            ReturnVal = false;
        }
        else {
            // Clear registers
            ANSELBCLR = WhichPin;
            ODCBCLR = WhichPin;
            CNPUBCLR = WhichPin;
            CNPDBCLR = WhichPin;
            
            // Configure input
            TRISBSET = WhichPin;
            CNCONBbits.ON = 1;
            CNENBSET = WhichPin;
        }
    } 
    else {
        ReturnVal = false;
    }
    return ReturnVal;
}