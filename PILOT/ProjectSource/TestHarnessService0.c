/****************************************************************************
 Module
   TestHarnessService0.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:26 jec     moves definition of ALL_BITS to ES_Port.h
 10/19/17 21:28 jec     meaningless change to test updating
 10/19/17 18:42 jec     removed referennces to driverlib and programmed the
                        ports directly
 08/21/17 21:44 jec     modified LED blink routine to only modify bit 3 so that
                        I can test the new new framework debugging lines on PF1-2
 08/16/17 14:13 jec      corrected ONE_SEC constant to match Tiva tick rate
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// This module
#include "../ProjectHeaders/TestHarnessService0.h"

// Hardware
#include <xc.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "terminal.h"
#include "bitdefs.h"

// Other libraries
#include "../../HALs/PIC32PortHAL.h"
#include "../../HALs/PIC32_AD_Lib.h"

/*----------------------------- Module Defines ----------------------------*/
#define JXPORT _Port_B
#define JXPIN _Pin_15
#define JXREAD PORTBbits.RB15
#define JXANPIN BIT9HI

#define JYPORT _Port_B
#define JYPIN _Pin_13
#define JYREAD PORTBbits.RB13
#define JYANPIN BIT11HI

#define NUMPINS 2

// Fwd: x=1023, Rev: x=0, Right: y=1023, Left: y=0
#define MAX 1023
#define XMID 509
#define YMID 503
#define MIN 0

// these times assume a 1 mS/tick timing
#define JOYSTICK_TIMEOUT 200     // 200 ms
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)

//#define PRINT_JOYSTICK // send data to terminal?
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static bool InitJoystick(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint32_t LastJoystickVal[NUMPINS];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService0

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitTestHarnessService0(uint8_t Priority)
{
    clrScrn();
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  if (!InitJoystick()) return false;

  /********************************************
   in here you write your initialization code
   *******************************************/

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
     PostTestHarnessService0

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostTestHarnessService0(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService0

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunTestHarnessService0(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
        ES_Timer_InitTimer(JOYSTICK_TIMER, JOYSTICK_TIMEOUT);
    }
    break;
    
    case ES_TIMEOUT:   // re-start timer & announce
    {
      #ifdef PRINT_JOYSTICK
        ADC_MultiRead(LastJoystickVal);
        printf("\rX: %d Y: %d\r\n", LastJoystickVal[0], LastJoystickVal[1]);
        ES_Timer_InitTimer(JOYSTICK_TIMER, JOYSTICK_TIMEOUT);
      #endif /* PRINT_JOYSTICK */
    }
    break;
    case ES_NEW_KEY:   // announce
    {
      printf("ES_NEW_KEY received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);
    }
    break;
    default:
    {}
     break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static bool InitJoystick()
{
    if (!PortSetup_ConfigureAnalogInputs(JXPORT,JXPIN)) return false;
    if (!PortSetup_ConfigureAnalogInputs(JYPORT,JYPIN)) return false;
    ADC_ConfigAutoScan(JXANPIN | JYANPIN, NUMPINS);
    ADC_MultiRead(LastJoystickVal);
    return true;
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

