/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef PilotService_H
#define PilotService_H

#include "ES_Types.h"

// Public Function Prototypes
typedef enum {
    PilotInitState, PilotWaiting, PilotRefueling
} PilotState_t;
bool InitPilotService(uint8_t Priority);
bool PostPilotService(ES_Event_t ThisEvent);
ES_Event_t RunPilotService(ES_Event_t ThisEvent);
bool joystickForceUpdate_Checker(void);

#endif /* PilotService_H */

