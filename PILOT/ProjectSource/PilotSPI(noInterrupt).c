/****************************************************************************
 Module
   PilotSPI.c

 Revision
   1.0.1

 Description
 SPI service for the PILOT

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// Hardware
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx170f256b.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "bitdefs.h"

// Other libraries
#include "../../HALs/PIC32PortHAL.h"
#include "../../HALs/PIC32_SPI_HAL.h"

// This module
#include "PilotSPI.h"

/*----------------------------- Module Defines ----------------------------*/
// Define I/O Ports
#define CONDISP_SS_PORT _Port_A
#define CONDISP_SS_PIN _Pin_0
#define CONDISP_SS LATAbits.LATA0

#define GASDISP_SS_PORT _Port_A
#define GASDISP_SS_PIN _Pin_1
#define GASDISP_SS LATAbits.LATA1

#define ACC_SS_PORT _Port_B
#define ACC_SS_PIN _Pin_9
#define ACC_SS LATBbits.LATB9

// Define SPI module and pin maps
#define SPI_MODULE SPI_SPI1
#define MISO SPI_RPB5
#define MOSI SPI_RPB8
#define BIT_TIME (uint32_t) 50e3         // 200 kHz
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void ConfigurePilotSPI(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static PilotSPIState_t CurrentState;
static uint8_t MyPriority;
static ES_Event_t DeferralQueue[3+1];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
 InitPilotSPI

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     Aaron Brown, 05/11/22
****************************************************************************/
bool InitPilotSPI(uint8_t Priority)
{
    ES_Event_t ThisEvent;
    MyPriority = Priority;
    //ES_InitDeferralQueueWith(DeferralQueue,ARRAY_SIZE(DeferralQueue));

    ConfigurePilotSPI();
    
    // post the initial transition event
    ThisEvent.EventType = ES_INIT;
    if (ES_PostToService(MyPriority, ThisEvent)) return true;
    else return false;
}

/****************************************************************************
 Function
 PostPilotSPI

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
 Aaron Brown, 5/11/22
****************************************************************************/
bool PostPilotSPI(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
 RunPilotSPI

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
 Aaron Brown, 5/11/22
****************************************************************************/
ES_Event_t RunPilotSPI(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    switch (ThisEvent.EventType)
    {
        default:
        {}
        break;
    }

    return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void ConfigurePilotSPI()
{
    /* Setup SPI1 at 500 kHz */
    SPISetup_BasicConfig(SPI_MODULE);                      // Basic config
    SPISetup_SetLeader(SPI_MODULE, SPI_SMP_MID);           // Set PIC as master mode, choose phase (typ. mid)
    SPISetup_SetBitTime(SPI_MODULE, BIT_TIME);              // Set requested bit time
    SPISetup_SetClockIdleState(SPI_MODULE, SPI_CLK_HI);    // Set clock to idle high
    SPISetup_SetActiveEdge(SPI_MODULE, SPI_SECOND_EDGE);   // Set second edge as active
    SPISetup_SetXferWidth(SPI_MODULE, SPI_16BIT);           // Set transfer width to 16 bit
    SPISetEnhancedBuffer(SPI_MODULE, true);                // Turn on enhanced buffer
    
    // SS, MISO, MOSI
    PortSetup_ConfigureDigitalOutputs(CONDISP_SS_PORT, CONDISP_SS_PIN); // SS line for CONCON Display
    PortSetup_ConfigureDigitalOutputs(GASDISP_SS_PORT, GASDISP_SS_PIN);   // SS line for GASCON Display
    PortSetup_ConfigureDigitalOutputs(ACC_SS_PORT, ACC_SS_PIN);   // SS line for accelerometer
    SPISetup_MapSSOutput(SPI_MODULE, SPI_NO_PIN);          // Don't map SS line since multiple followers
    SPISetup_MapSDInput(SPI_MODULE, MISO);                 // Map MISO pin
    SPISetup_MapSDOutput(SPI_MODULE, MOSI);                // Map MOSI pin

    // Turn on SPI
    SPISetup_EnableSPI(SPI_MODULE);

    // Deactivate SS lines
    CONDISP_SS = 1;
    GASDISP_SS = 1;
    ACC_SS = 1;

    // Clear SPI buffer
    SPIOperate_clearBuffer(SPI_MODULE);
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

