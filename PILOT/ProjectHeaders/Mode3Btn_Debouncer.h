/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef Mode3Btn_Debouncer_H
#define Mode3Btn_Debouncer_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "../../HALS/Debouncer_Common.h"

// typedefs for the states, handled in Debouncer_Common

// Public Function Prototypes

bool InitMode3Btn_DebounceFSM(uint8_t Priority);
bool PostMode3Btn_DebounceFSM(ES_Event_t ThisEvent);
ES_Event_t RunMode3Btn_DebounceFSM(ES_Event_t ThisEvent);
Debouncer_state_t QueryMode3Btn_DebounceSM(void);

// Event Checker
bool Mode3Btn_Debouncer_EventChecker(void);

#endif /* Mode3Btn_Debouncer_H */

