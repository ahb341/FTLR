/****************************************************************************

  Header file for Test Harness Service0
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef PilotSPI_H
#define PilotSPI_H

#include "ES_Events.h"
#include "ES_Port.h" 

// typedefs for the states
typedef enum
{
    InitPState, ConconInit, GasconInit, StartDisplay, AccInit, Waiting, 
            SendingConconDisplay, SendingGasconDisplay, 
            ReadingAcc
} PilotSPIState_t;

// Public Function Prototypes
bool InitPilotSPI(uint8_t Priority);
bool PostPilotSPI(ES_Event_t ThisEvent);
ES_Event_t RunPilotSPI(ES_Event_t ThisEvent);
PilotSPIState_t QueryPilotSPI(void);
bool finishedSPI_transfer(void);
bool accelForceUpdate_Checker(void);

#endif /* PilotSPI_H */

