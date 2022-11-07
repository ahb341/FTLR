/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef TUG_CommComm_H
#define TUG_CommComm_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, WaitingForPairRequest, 
  WaitingForControlPacket, Paired,
} TUG_CommComm_t;

// Public Function Prototypes

bool InitTUGCommComm(uint8_t Priority);
bool PostTUGCommComm(ES_Event_t ThisEvent);
ES_Event_t RunTUGCommComm(ES_Event_t ThisEvent);
TUG_CommComm_t QueryTUGCommComm(void);

#endif /* TUG_CommComm_H */
