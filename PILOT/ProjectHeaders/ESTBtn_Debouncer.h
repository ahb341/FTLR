/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef ESTBtn_Debouncer_H
#define ESTBtn_Debouncer_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "../../HALS/Debouncer_Common.h"

// typedefs for the states, handled in Debouncer_Common

// Public Function Prototypes

bool InitESTBtn_DebounceFSM(uint8_t Priority);
bool PostESTBtn_DebounceFSM(ES_Event_t ThisEvent);
ES_Event_t RunESTBtn_DebounceFSM(ES_Event_t ThisEvent);
Debouncer_state_t QueryESTBtn_DebounceSM(void);

// Event Checker
bool ESTBtn_Debouncer_EventChecker(void);

#endif /* ESTBtn_Debouncer_H */

