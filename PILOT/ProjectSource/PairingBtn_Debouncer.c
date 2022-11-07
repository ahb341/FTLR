/****************************************************************************
 Module
   XXX_Debouncer.c

 Revision
   1.0.1

 Description
    Deobuncing code for Team 0 Lab 10

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "PairingBtn_Debouncer.h"
#include "../../HALs/PIC32PortHAL.h"
#include "ES_Timers.h"
#include "Pilot_CommComm.h"

/*----------------------------- Module Defines ----------------------------*/

#define bouncePin   _Pin_2  //RAO
#define bouncePort  _Port_A
#define bounceRead  PORTAbits.RA2

#define bounceTimer PairingBtn_Debounce_Timer // locally store it for ease of changing it around

// debug control, comment out to stop prints
//#define DEBOUNCE_PRINT // print when a button is debounced?
// #define Announce_ShortLong // print when a short versus long press

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static Debouncer_state_t CurrentState;

static bool stablized = false; //did we get one stable reading?

// event checker value
static bool lastState;

// how long has it been pressed?
static uint16_t pressedTimeStamp; // when the button when down

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitPairingBtn_DebounceFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  // put us into the Initial PseudoState
  CurrentState = Debounce_InitPState;

  //setup pin as input
  PortSetup_ConfigurePullUps(bouncePort, bouncePin);

  //get initial port state
  lastState = bounceRead;

  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostTemplateFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostPairingBtn_DebounceFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunPairingBtn_DebounceFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case Debounce_InitPState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // read initial state, make us debounce the first motion too
        bool currVal = bounceRead; // get state at start
        if (true == currVal) 
        {
          // we are high, assume it's NOT stable
          CurrentState = high_testing;
          ES_Timer_InitTimer(bounceTimer, DEBOUNCE_TIME);
        }
        else
        {
          // low, test again
          CurrentState = low_testing;
          ES_Timer_InitTimer(bounceTimer, DEBOUNCE_TIME);
        }
        #ifdef DEBOUNCE_PRINT
            clrScrn();
        #endif
      }
    }
    break;

    //starting low
    case low_stable:        // if stable low
    {
      switch (ThisEvent.EventType)
      {
        case ES_ButtonToggle:  //If event is a pin change
        {   
          // check if rising edge
          if (1 == ThisEvent.EventParam) 
          {
            //start testing for stable high
            CurrentState = high_testing;
            ES_Timer_InitTimer(bounceTimer, DEBOUNCE_TIME);
            
            #ifdef DEBOUNCE_PRINT
              printf("low stable, testing high?\n\r");
            #endif
          }
          else 
          {
            //this should be imposible, but case is here in case...
            ;
          }

        }
        break;

        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;

    case low_testing:
    {
      switch (ThisEvent.EventType)
      {
      case ES_ButtonToggle:
        {
          // ignore this, we assume it's going to change a bunch lol
          break;
        }

      case ES_TIMEOUT:
      {
        if (bounceTimer == ThisEvent.EventParam)
        {
          //we got a timeout, if we are at the same state, we should be stable!
          bool currVal = bounceRead; // get state at start
          if (0 == currVal) 
          {
            //we got a low, assume it's stable!
            CurrentState = low_stable;

            if (stablized)
            {
              //TODO: Post relevant here!
              pressedTimeStamp = ES_Timer_GetTime(); // record time
            }
            else 
            {
              //first time running through
              stablized = true;
            }

            #ifdef DEBOUNCE_PRINT
              printf("Pairing Button is now LOW\n\r");
            #endif
          } 
          else 
          {
            //got a high, it should be stably high again
            CurrentState = high_stable; //exit time check loop
          }

        }
      }
      break;
      
      default:
        break;
      }
    }
    break;

    //starting high
    case high_stable:        // if stable high
    {
      switch (ThisEvent.EventType)
      {
        case ES_ButtonToggle:  //If event is a pin change
        {   
          // check if falling edge
          if (0 == ThisEvent.EventParam) 
          {
            //start testing for stable high
            CurrentState = low_testing;
            ES_Timer_InitTimer(bounceTimer, DEBOUNCE_TIME);
            
            #ifdef DEBOUNCE_PRINT
              printf("high stable, testing low?\n\r");
            #endif
          }
          else 
          {
            //this should be imposible, but case is here in case...
            ;
          }

        }
        break;

        default:
          ;
      }  // end switch on CurrentEvent
    }
    break;

    case high_testing:
    {
      switch (ThisEvent.EventType)
      {
      case ES_ButtonToggle:
        {
          // ignore this, we assume it's going to change a bunch lol
          break;
        }

      case ES_TIMEOUT:
      {
        if (bounceTimer == ThisEvent.EventParam)
        {
          //we got a timeout, if we are at the same state, we should be stable!
          bool currVal = bounceRead; // get state at start
          if (1 == currVal) 
          {
            //we got a high, assume it's stable!
            CurrentState = high_stable;

            if (stablized)
            {
              //TODO: Post relevant here!
              // check how long it was pressed
              uint16_t pressLength = ES_Timer_GetTime() - pressedTimeStamp;
              if (LONG_PRESS_THRESHOLD > pressLength) 
              {
                // pressed for less than full threshold
                // Increment pairing TUG number
                incrementPairingIndex();
                #ifdef Announce_ShortLong 
                  printf("Pairing button Short press! \n\r");
                #endif /* Announce_ShortLong */
              }
              else
              {
                // long press, start pairing!
                PostPilotCommComm((ES_Event_t) {ES_StartPairing, 0});
                #ifdef Announce_ShortLong 
                  printf("Pairing button Long press! \n\r");
                #endif /* Announce_ShortLong */
              }
            }
            else 
            {
              //first time running through
              stablized = true;
            }

            #ifdef DEBOUNCE_PRINT
              printf("Pairing Button is now HIGH\n\r");
            #endif
          } 
          else 
          {
            //got a low, it should be stably low again
            CurrentState = low_stable; //exit time check loop
          }

        }
      }
      break;
      
      default:
        break;
      }
    }
    break;

    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
Debouncer_state_t QueryPairingBtn_DebounceSM(void)
{
  return CurrentState;
}


// Event Checker!

bool PairingBtn_Debouncer_EventChecker(void)
{
  bool returnVal = false;
  bool currVal = bounceRead; //get current value
  if (lastState != currVal) {
    //weesa have a change!
    ES_PostToService(MyPriority, (ES_Event_t) {ES_ButtonToggle, currVal});
    returnVal = true; //there was an event!
  }
  lastState = currVal; //save it for next time
  return returnVal;
}

/***************************************************************************
 private functions
 ***************************************************************************/

