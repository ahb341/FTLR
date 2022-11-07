#ifndef PIC32PortHAL
#define PIC32PortHAL

#include <stdbool.h>

/****************************************************************************
This enum defines a data type for the possible ports to be configured.
Note: explicit assignment used to eliminate any possible compiler dependency
****************************************************************************/
typedef enum
{
  _Port_A = 0,
  _Port_B = 1,
}PortSetup_Port_t;

/****************************************************************************
This enum defines a data type for the possible pins to be configured.
the values are a constant with a single '1' in the chosen bit position 
****************************************************************************/
typedef enum
{
  _Pin_0 = (1<<0),
  _Pin_1 = (1<<1),
  _Pin_2 = (1<<2),
  _Pin_3 = (1<<3),
  _Pin_4 = (1<<4),
  _Pin_5 = (1<<5),
  _Pin_6 = (1<<6),
  _Pin_7 = (1<<7),
  _Pin_8 = (1<<8),
  _Pin_9 = (1<<9),
  _Pin_10 = (1<<10),
  _Pin_11 = (1<<11),
  _Pin_12 = (1<<12),
  _Pin_13 = (1<<13),
  _Pin_14 = (1<<14),
  _Pin_15 = (1<<15)
}PortSetup_Pin_t;

/****************************************************************************
 Function
    PortSetup_ConfigureDigitalInputs

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin to be configured as digital inputs

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port as digital inputs, disabling analog input(s).
Example
   PortSetup_ConfigureDigitalInputs(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigureDigitalInputs( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);
 
/****************************************************************************
 Function
    PortSetup_ConfigureDigitalOutputs

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin(s) to be configured as digital outputs

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port as digital outputs
Example
   PortSetup_ConfigureDigitalOutputs(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigureDigitalOutputs( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);

/****************************************************************************
 Function
    PortSetup_ConfigureAnalogInputs

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin(s) to be configured as analog inputs

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port as analog inputs
Example
   PortSetup_ConfigureAnalogInputs(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigureAnalogInputs( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);

/****************************************************************************
 Function
    PortSetup_ConfigurePullUps

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin(s) to be configured with weak pull-ups

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port with weak pull-ups
Example
   PortSetup_ConfigurePullUps(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigurePullUps( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);
 
/****************************************************************************
 Function
    PortSetup_ConfigurePullDowns

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin(s) to be configured with weak pull-downs

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port with weak pull-downs. 
Example
   PortSetup_ConfigurePullDowns(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigurePullDowns( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);

/****************************************************************************
 Function
    PortSetup_ConfigureOpenDrain

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin(s) to be configured as open drain outputs

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port as open drain outputs
Example
   PortSetup_ConfigureOpenDrain(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigureOpenDrain( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);

/****************************************************************************
 Function
    PortSetup_ConfigureChangeNotification

 Parameters
   PortSetup_Port_t: the port to be configured
   PortSetup_Pin_t: the pin(s) to be enabled for change notification

 Returns
   bool: true if port and pins represent legal ports and pins; otherwise, false

 Description
   Configures the specified pin(s) on the specified port to enable change notifications. If any bits are set in the PortSetup_Pin_t parameter, then change notifications are enabled. If that parameter is 0, then change notifications are disabled globally.
Example
   PortSetup_ConfigureChangeNotification(_Port_A, _Pin_0 | _Pin_1);
****************************************************************************/
bool PortSetup_ConfigureChangeNotification( PortSetup_Port_t WhichPort, PortSetup_Pin_t WhichPin);

#endif /* PIC32PortHAL */