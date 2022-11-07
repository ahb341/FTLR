/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef PumpService_H
#define PumpService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  PumpInitPState, FuelRemaining, WaitingForRefuel
}PumpState_t;

// Public Function Prototypes

bool InitPumpService(uint8_t Priority);
bool PostPumpService(ES_Event_t ThisEvent);
ES_Event_t RunPumpService(ES_Event_t ThisEvent);
PumpState_t QueryPumpService(void);

void UpdateThrustVectors(int8_t NewCtrlX, int8_t NewCtrlY, int8_t NewCtrlZ);
uint8_t getRemainingFuel();

#endif /* PumpService_H */

