/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef InputSwBtn_Debouncer_H
#define InputSwBtn_Debouncer_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "../../HALS/Debouncer_Common.h"

// typedefs for the states, handled in Debouncer_Common

// Public Function Prototypes

bool InitInputSwBtn_DebounceFSM(uint8_t Priority);
bool PostInputSwBtn_DebounceFSM(ES_Event_t ThisEvent);
ES_Event_t RunInputSwBtn_DebounceFSM(ES_Event_t ThisEvent);
Debouncer_state_t QueryInputSwBtn_DebounceSM(void);

// Event Checker
bool InputSwBtn_Debouncer_EventChecker(void);

#endif /* InputSwBtn_Debouncer_H */

