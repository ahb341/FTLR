/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef Pilot_CommComm_H
#define Pilot_CommComm_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  CC_InitPState, AttemptingToPair, Paired,
} Pilot_CommComm_t;

// Public Function Prototypes

bool InitPilotCommComm(uint8_t Priority);
bool PostPilotCommComm(ES_Event_t ThisEvent);
ES_Event_t RunPilotCommComm(ES_Event_t ThisEvent);
Pilot_CommComm_t QueryPilotCommComm(void);

// access Pairing data
void incrementPairingIndex();
void setToPairingIndex(uint8_t newVal); // set to a specific value
uint8_t getTUGpairingIndex();
uint8_t getTUGlatchedIndex();

#define TUG_Unpaired_Index  165 // this value when we are unpaired to a TUG



#endif /* Pilot_CommComm_H */
