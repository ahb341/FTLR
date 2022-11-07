/****************************************************************************
 Module
     ES_EventCheckWrapper.h
 Description
     This is a wrapper header file for all of the header files that include
     prototypes for event checking functions.
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 12/19/16 20:12 jec      Started coding
*****************************************************************************/

#ifndef ES_EventCheckWrapper_H
#define ES_EventCheckWrapper_H

// This is the header for the event checkers for the template project
#include "EventCheckers.h"
#include "PilotSPI.h"
#include "accelerometer.h"
// Here you would #include the header files for any other modules that
// contained event checking functions
#include "PairingBtn_Debouncer.h"
#include "ESTBtn_Debouncer.h"
#include "Mode3Btn_Debouncer.h"
#include "InputSwBtn_Debouncer.h"
#include "PilotSPI.h"
#include "PilotService.h"

#endif  // ES_EventCheckWrapper_H
