/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef PairingBTN_Debouncer_H
#define PairingBTN_Debouncer_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "../../HALS/Debouncer_Common.h"

// typedefs for the states, handled in Debouncer_Common

// Public Function Prototypes

bool InitPairingBtn_DebounceFSM(uint8_t Priority);
bool PostPairingBtn_DebounceFSM(ES_Event_t ThisEvent);
ES_Event_t RunPairingBtn_DebounceFSM(ES_Event_t ThisEvent);
Debouncer_state_t QueryPairingBtn_DebounceSM(void);

// Event Checker
bool PairingBtn_Debouncer_EventChecker(void);

#endif /* PairingBTN_Debouncer_H */

