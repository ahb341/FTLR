/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef Pilot_XBee_H
#define Pilot_XBee_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  P_XBee_Init, P_XBee_ready 
}Pilot_XBee_states_t;

// Public Function Prototypes

bool InitPilotXBee(uint8_t Priority);
bool PostPilotXBee(ES_Event_t ThisEvent);
ES_Event_t RunPilotXBee(ES_Event_t ThisEvent);
Pilot_XBee_states_t QueryPilotXBee(void);
void setControl(int8_t x, int8_t y, int8_t yaw);

//configuration 
void ConfigUART(void);

// create messages 
void constructATEcho();
void constructSetPairingReq(uint16_t TUGaddr);
void constructPairingReq(); // overload, use stored address!
void constructControlFrame();

// write values in
void setControlValues(int8_t newXControl, int8_t newYControl,
                      int8_t newYawControl);
void doTheRefuel(); // set that we should refuel
void setMode3Value(uint8_t newMode3);

// get fuel level
uint8_t getFuelLevel();

//update activity LEDs
void updateActivityLEDs(void);


#endif /* Pilot_XBee_H */

