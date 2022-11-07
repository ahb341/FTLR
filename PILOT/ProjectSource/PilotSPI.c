/****************************************************************************
 Module
   PilotSPI.c

 Revision
   1.0.1

 Description
 SPI service for the PILOT. Keyboard input writes to the LED matrix displays.
 Reads the accelerometer every 20 ms.

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
#include "../../HALs/spi_standards.h"
#include "../../HALs/dm_display.h"
#include "accelerometer.h"
#include "Pilot_XBee.h"     

// This module
#include "PilotSPI.h"

/*----------------------------- Module Defines ----------------------------*/
// Accelerometer
#define ACC_BASE_PERIOD 20
#define PRINT_TIMEOUT 2000
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

// SPI interrupt priority
#define SPI1_IP 7
#define SPI1_IPL IPL7SOFT

// SS line idle and active values
#define IDLE 1
#define ACTIVE 0

// Display
#define CHAR_ROW 3
#define COL00 28
#define COL01 24
#define COL10 20
#define COL11 16
#define COL20 12
#define COL21 8
#define COL30 4
#define COL31 0
#define EST_DEFAULT     'T'
#define PAIRED_DEFAULT  'U'
#define TEAM_DEFAULT    '6'
#define INPUT_DEFAULT1  'A'
#define INPUT_DEFAULT2  '2'
#define FUEL_DEFAULT    'F255'
#define NUM_DEFAULT     '0'

#define Mode3_DISP_COL  (COL31 + 4)
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void ConfigurePilotSPI(void);
static void AccInitHandler(ES_Event_t ThisEvent);
static void ConconDisplayUpdate(); // move inputs to static variable
static void GasconDisplayUpdate(unsigned char Fuel0, unsigned char Fuel1, \
        unsigned char Fuel2, unsigned char Num);
static void StartDisplayHandler(ES_Event_t ThisEvent);
void Mode3DisplayUpdate(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static PilotSPIState_t CurrentState;
static uint8_t MyPriority;
static ES_Event_t DeferralQueue[3+1];
static bool UpdateFinished;

typedef enum {
    CONCON0, CONCON1, GASCON0, GASCON1
} StartDisplayStep_t;
static StartDisplayStep_t StartDisplayStep;
static unsigned char ConconDisplay[5] = {EST_DEFAULT,PAIRED_DEFAULT,TEAM_DEFAULT,
                                         INPUT_DEFAULT1, INPUT_DEFAULT2};
static unsigned char GasconDisplay[4] = {'2','5','5','0'};
static uint8_t bitIndex = 0; // mode 3 index
static uint8_t mode3Control = 0; 

// somewhat jankily force accel timer to keep running
static uint16_t lastAccelWrite = 0; // how long since we wrote to accel?

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
    clrScrn();
    ES_Event_t ThisEvent;
    MyPriority = Priority;

    ES_InitDeferralQueueWith(DeferralQueue,ARRAY_SIZE(DeferralQueue));
    ConfigurePilotSPI();
    CurrentState = InitPState;

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
    ReturnEvent.EventType = ES_NO_EVENT;

    switch (CurrentState)
    {
        case InitPState:
        {
            if (ThisEvent.EventType == ES_INIT)
            {
                printf("\rES_INIT received in PilotSPI service \r\n");
                UpdateFinished = false;
                CurrentState = AccInit;
                ACC_SS = ACTIVE;
                UpdateFinished = Accel_TakeInitUpdateStep();
                IEC1bits.SPI1TXIE = 1;
            }
        }
        break;
        
        case AccInit:
        {
            AccInitHandler(ThisEvent);
        }
        break;
        
        case ConconInit:
        {
            if (ThisEvent.EventType == PSPI_XFER_DONE)
            {   
                if (UpdateFinished)
                {
                    CurrentState = GasconInit;
                    UpdateFinished = false;
                    PostPilotSPI((ES_Event_t){PSPI_XFER_DONE,0});
                }
                else
                {
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = DM_TakeInitDisplayStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
            }
        }
        break;
        
        case GasconInit:
        {
            if (ThisEvent.EventType == PSPI_XFER_DONE)
            {   
                if (UpdateFinished)
                {
                    CurrentState = StartDisplay;
                    StartDisplayStep = CONCON0;
                    PostPilotSPI((ES_Event_t) {PSPI_XFER_DONE,0});
                }
                else
                {
                    GASDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = DM_TakeInitDisplayStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
            }
        }
        break;
        
        case StartDisplay:
        {
            StartDisplayHandler(ThisEvent);
        }
        break;
        
        case Waiting:
        {
            switch (ThisEvent.EventType)
            {
                case (PSPI_DRIVE_MODE):
                {
                    ConconDisplay[0] = ThisEvent.EventParam;
                    ConconDisplayUpdate();
                    CurrentState = SendingConconDisplay; // Move to sending state
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Concon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;

                case (PSPI_INPUT_MODE):
                {
                    ConconDisplay[3] = ThisEvent.EventParam >> 8; // first char
                    ConconDisplay[4] = ThisEvent.EventParam & 0xFF; // second char
                    ConconDisplayUpdate();
                    CurrentState = SendingConconDisplay; // Move to sending state
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Concon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;

                case (PSPI_Activity_Update):
                {
                    // updateActivityLEDs(); // add activity LEDs
                    ConconDisplayUpdate();
                    CurrentState = SendingConconDisplay; // Move to sending state
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Concon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;
                
                case (PSPI_PAIR_UNPAIR):
                {
                    ConconDisplay[1] = (unsigned char) ThisEvent.EventParam;
                    ConconDisplayUpdate();
                    CurrentState = SendingConconDisplay; // Move to sending state
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Concon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;
                
                case (PSPI_TEAM_SELECT):
                {
                    printf("\rPSPI TEAM SELECT\r\n");
                    ConconDisplay[2] = (unsigned char) ThisEvent.EventParam;
                    ConconDisplayUpdate();
                    CurrentState = SendingConconDisplay; // Move to sending state
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Concon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;

                case (PSPI_Mode3_Update):
                {
                    // update the specific sections
                    bitIndex = ThisEvent.EventParam >> 8; // just upper 8 bits
                    mode3Control = ThisEvent.EventParam & 0xFF; // just lower 8
                    ConconDisplayUpdate();
                    CurrentState = SendingConconDisplay; // Move to sending state
                    CONDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Concon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;
                
                case (PSPI_FUEL_UPDATE):
                {
                    int fuel = ThisEvent.EventParam;
                    char buffer [3];
                    itoa (buffer,fuel,10);
                    if (fuel > 99)
                    {
                        GasconDisplay[0] = buffer[0];
                        GasconDisplay[1] = buffer[1];
                        GasconDisplay[2] = buffer[2];
                    }
                    else if (fuel > 9)
                    {
                        GasconDisplay[0] = '0';
                        GasconDisplay[1] = buffer[0];
                        GasconDisplay[2] = buffer[1];
                    }
                    else
                    {
                        GasconDisplay[0] = '0';
                        GasconDisplay[1] = '0';
                        GasconDisplay[2] = buffer[0];
                    }
                    GasconDisplayUpdate(GasconDisplay[0],GasconDisplay[1],\
                            GasconDisplay[2],GasconDisplay[3]);
                    CurrentState = SendingGasconDisplay;
                    GASDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Gascon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;
                }
                break;
                
                case (PSPI_DANCE_UPDATE):
                {
                    int num = ThisEvent.EventParam;
                    char buffer [3];
                    itoa (buffer,num,10);
                    GasconDisplay[3] = buffer[0];
                    GasconDisplayUpdate(GasconDisplay[0],GasconDisplay[1],\
                            GasconDisplay[2],GasconDisplay[3]);
                    CurrentState = SendingGasconDisplay;
                    GASDISP_SS = ACTIVE; //activate SS line
                    UpdateFinished = Gascon_TakeDisplayUpdateStep();
                    IEC1bits.SPI1TXIE = 1;
                }
                break;
                
                case (PSPI_ACC_UPDATE_REQ):
                {
                    //clear SPI buffer!
                    SPIOperate_clearBuffer(SPI_MODULE);
                    CurrentState = ReadingAcc; //move to updater state
                    ACC_SS = ACTIVE; //select!
                    Accel_requestAccelData();
                    lastAccelWrite = ES_Timer_GetTime(); // timestamp transmission
                    IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                }
                break;
                
                case (ES_TIMEOUT):
                {
                    if (SPIUpdate_TIMER == ThisEvent.EventParam) //update ready?
                    {
                        ES_Event_t toPost = {PSPI_ACC_UPDATE_REQ,0};
                        PostPilotSPI(toPost); //request a thing
                        //keep accel up to date
                        ES_Timer_InitTimer(SPIUpdate_TIMER,ACC_BASE_PERIOD);
                    }
                    else if (PRINT_TIMER == ThisEvent.EventParam)
                    {
                        //printf("\rACCX: %d        ACCY: %d         ACCZ: %d\r\n", GetX(), GetY(), GetZ());
                        ES_Timer_InitTimer(PRINT_TIMER,PRINT_TIMEOUT);
                    }
                }
                break;
                
                
                default:
                {
                    ES_RecallEvents(MyPriority,DeferralQueue);
                }
                break;
            }
        }
        break;
        
        case SendingConconDisplay:
        {
            switch (ThisEvent.EventType)
            {
                case (PSPI_XFER_DONE):
                {   
                    if (UpdateFinished)
                    {
                        CurrentState = Waiting;
                        UpdateFinished = false;
                        ES_RecallEvents(MyPriority,DeferralQueue);
                    }
                    else
                    {
                        CONDISP_SS = ACTIVE; //activate SS line
                        UpdateFinished = Concon_TakeDisplayUpdateStep();
                        IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                    }
                }
                break;
                
                default:
                {
                    ES_DeferEvent(DeferralQueue,ThisEvent);
                }
                break;
            }
        }
        break;
        
        case SendingGasconDisplay:
        {
            switch (ThisEvent.EventType)
            {
                case (PSPI_XFER_DONE):
                {   
                    if (UpdateFinished)
                    {
                        CurrentState = Waiting;
                        UpdateFinished = false;
                        ES_RecallEvents(MyPriority,DeferralQueue);
                    }
                    else
                    {
                        GASDISP_SS = ACTIVE; //activate SS line
                        UpdateFinished = Gascon_TakeDisplayUpdateStep();
                        IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                    }
                }
                break;
                
                default:
                {
                    ES_DeferEvent(DeferralQueue,ThisEvent);
                }
                break;
            }
        }
        break;
        
        case ReadingAcc:
        {
            switch (ThisEvent.EventType)
            {
                case PSPI_XFER_DONE:
                {
                    //capture data, see if we're done
                    if(0 == Accel_sequesterAccelData())
                    {
                        //force an update on next cycle!
                        PostPilotSPI((ES_Event_t) {ES_TIMEOUT, SPIUpdate_TIMER});
                    }
                    else //we did finish!
                    {
                        CurrentState = Waiting; //return to main wait
                        ES_RecallEvents(MyPriority,DeferralQueue); //see if we missed any
                        ES_Timer_InitTimer(SPIUpdate_TIMER, ACC_BASE_PERIOD);
                        // ES_Timer_InitTimer(JOYSTICK_TIMER, 1000);
                    }
                }
                break;

                case ES_TIMEOUT:
                {
                    if (SPIUpdate_TIMER == ThisEvent.EventParam) //update ready?
                    {
                        ACC_SS = ACTIVE; //select!
                        Accel_requestAccelData();
                        IEC1bits.SPI1TXIE = 1;          // Enable transmit interrupt
                    }
                }
                break;

                default:
                {
                    ES_DeferEvent(DeferralQueue,ThisEvent);                    
                }
                break;
            }
        }
        break;
        
        default:
        {
            ES_DeferEvent(DeferralQueue,ThisEvent);
        }
        break;
    }

    return ReturnEvent;
}

bool accelForceUpdate_Checker(void) {
    if (2*ACC_BASE_PERIOD < (lastAccelWrite - ES_Timer_GetTime())) 
    {
        // been too long, ask for update
        PostPilotSPI((ES_Event_t) {PSPI_ACC_UPDATE_REQ, 0});
        ES_Timer_InitTimer(SPIUpdate_TIMER,1); // restart timer
        printf("forced accel timer on \n\r");
        return true;
    }
    return false;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void ConfigurePilotSPI()
{
    //Disable all interrupts
    IEC1CLR = _IEC1_SPI1EIE_MASK;
    IEC1CLR = _IEC1_SPI1RXIE_MASK;
    IEC1CLR = _IEC1_SPI1TXIE_MASK;
    
    // Clear any existing flags
    IFS1CLR = _IFS1_SPI1EIF_MASK;
    IFS1CLR = _IFS1_SPI1RXIF_MASK;
    IFS1CLR = _IFS1_SPI1TXIF_MASK;
    
    IPC7bits.SPI1IP = SPI1_IP;                        //Set interrupt priority level
    
    SPI1STATCLR = _SPI1STAT_SPIROV_MASK;        // Clear the Overflow
    SPI1CONbits.STXISEL = 0b00;                 // TX int. thrown when last shifted out
    //SPI1CONbits.SRXISEL = 0b01;               // RX int. thrown when not empty
            
    /* Setup SPI1 at 500 kHz */
    SPISetup_BasicConfig(SPI_MODULE);                      // Basic config
    SPISetup_SetLeader(SPI_MODULE, SPI_SMP_MID);           // Set PIC as master mode, choose phase (typ. mid)
    SPISetup_SetBitTime(SPI_MODULE, BIT_TIME);              // Set requested bit time
    SPISetup_SetClockIdleState(SPI_MODULE, SPI_CLK_HI);    // Set clock to idle high
    SPISetup_SetActiveEdge(SPI_MODULE, SPI_SECOND_EDGE);   // Set second edge as active
    SPISetup_SetXferWidth(SPI_MODULE, SPI_16BIT);           // Set transfer width to 8 bit
    SPISetEnhancedBuffer(SPI_MODULE, true);                // Turn on enhanced buffer
    
    // SS, MISO, MOSI
    PortSetup_ConfigureDigitalOutputs(CONDISP_SS_PORT, CONDISP_SS_PIN); // SS line for CONCON Display
    PortSetup_ConfigureDigitalOutputs(GASDISP_SS_PORT, GASDISP_SS_PIN);   // SS line for GASCON Display
    PortSetup_ConfigureDigitalOutputs(ACC_SS_PORT, ACC_SS_PIN);   // SS line for accelerometer
    SPISetup_MapSSOutput(SPI_MODULE, SPI_NO_PIN);          // Don't map SS line since multiple followers
    SPISetup_MapSDInput(SPI_MODULE, MISO);                 // Map MISO pin
    SPISetup_MapSDOutput(SPI_MODULE, MOSI);                // Map MOSI pin

    // Deactivate SS lines
    CONDISP_SS = IDLE;
    GASDISP_SS = IDLE;
    ACC_SS = IDLE;

    // Clear SPI buffer
    SPIOperate_clearBuffer(SPI_MODULE);
    
    // Turn on SPI
    SPISetup_EnableSPI(SPI_MODULE);
}

static void AccInitHandler (ES_Event_t ThisEvent)
{
    switch (ThisEvent.EventType)
    {             
        case PSPI_XFER_DONE:
        {
            if (UpdateFinished)
            {
                printf("\rACC UPDATE FINISHED\r\n");
                CurrentState = ConconInit;
                UpdateFinished = false;
                PostPilotSPI((ES_Event_t){PSPI_XFER_DONE,0});
            }
            else
            {
                if (Acc_CalibrationData == Accel_Init_needsData()) //save some data!
                {
                    Accel_sequesterAccelData();
                    ACC_SS = ACTIVE;
                    UpdateFinished = Accel_TakeInitUpdateStep();
                    IEC1bits.SPI1TXIE = 1;
                }
                else if (Acc_ReadTrimRegisters == Accel_Init_needsData())
                { 
                    //pull data from trim regs
                    readTrimRegister(); //read data
                    ACC_SS = ACTIVE;
                    UpdateFinished = Accel_TakeInitUpdateStep();
                    IEC1bits.SPI1TXIE = 1;
                }
                else
                {
                    printf("\rACC INIT STEP\r\n");
                    ACC_SS = ACTIVE;
                    UpdateFinished = Accel_TakeInitUpdateStep();
                    IEC1bits.SPI1TXIE = 1;
                }
            }
        }
        break;
    }
}

static void ConconDisplayUpdate()
{
    char Est    = ConconDisplay[0];
    char Pair   = ConconDisplay[1];
    char ToPair = ConconDisplay[2];
    char Input1 = ConconDisplay[3];
    char Input2 = ConconDisplay[4];
    Concon_ClearDisplayBuffer();
    uint8_t RowOrigin = CHAR_ROW;
    Concon_AddChar2BufferPos(Est,RowOrigin,COL00);
    Concon_AddChar2BufferPos(Pair,RowOrigin,COL20);
    Concon_AddChar2BufferPos(ToPair,RowOrigin,COL30);
    Concon_AddChar2BufferPos(Input1,RowOrigin,COL10);
    Concon_AddChar2BufferPos(Input2,RowOrigin,COL11);
    Mode3DisplayUpdate(); // also update mode 3 stuff
    updateActivityLEDs();
}

static void GasconDisplayUpdate(unsigned char Fuel0, unsigned char Fuel1, \
        unsigned char Fuel2, unsigned char Num)
{
    Gascon_ClearDisplayBuffer();
    uint8_t RowOrigin = CHAR_ROW;
    Gascon_AddChar2BufferPos('F',RowOrigin,COL00);
    Gascon_AddChar2BufferPos(Fuel0,RowOrigin,COL01);
    Gascon_AddChar2BufferPos(Fuel1,RowOrigin,COL10);
    Gascon_AddChar2BufferPos(Fuel2,RowOrigin,COL11);
    Gascon_AddChar2BufferPos(Num,RowOrigin,COL31);
}



static void StartDisplayHandler(ES_Event_t ThisEvent)
{
    switch (ThisEvent.EventType)
            {
                case (PSPI_XFER_DONE):
                {
                    switch (StartDisplayStep)
                    {
                        case (CONCON0):
                        {
                            ConconDisplayUpdate();
                            Mode3DisplayUpdate(); // init mode 3 stuff
                            PostPilotSPI((ES_Event_t) {PSPI_XFER_DONE,0});
                            StartDisplayStep++;
                        }
                        break;
                        
                        case (CONCON1):
                        {
                            CONDISP_SS = ACTIVE; //activate SS line
                            UpdateFinished = Concon_TakeDisplayUpdateStep();
                            IEC1bits.SPI1TXIE = 1;
                            if (UpdateFinished) StartDisplayStep++;
                        }
                        break;
                        
                        case (GASCON0):
                        {
                            GasconDisplayUpdate('2','5','5','0');
                            PostPilotSPI((ES_Event_t) {PSPI_XFER_DONE,0});
                            StartDisplayStep++;
                        }
                        break;
                        
                        case (GASCON1):
                        {
                            GASDISP_SS = ACTIVE; //activate SS line
                            UpdateFinished = Gascon_TakeDisplayUpdateStep();
                            IEC1bits.SPI1TXIE = 1;
                            
                            if (UpdateFinished)
                            {
                                StartDisplayStep = CONCON0;
                                CurrentState = Waiting;
                                ES_Timer_InitTimer(SPIUpdate_TIMER,ACC_BASE_PERIOD);
                                ES_Timer_InitTimer(PRINT_TIMER,PRINT_TIMEOUT);
                            }
                        }
                        break;
                    }
                }
                break;
            }
}

void Mode3DisplayUpdate()
{
    // printf("PSPI_Mode3_Update, ind: %d, ctrl: %x\n\r", bitIndex, mode3Control);
    for (uint8_t i = 0; i < 8; i++)
    {
        // add the indexing bit
        Concon_AddBit2BufferPos(i == bitIndex, i, Mode3_DISP_COL-1);
        Concon_AddBit2BufferPos(i == bitIndex, i, Mode3_DISP_COL-2);
        
        // Add control bits
        // printf("\t row: %d, ctrl: %d, indVal: %d\n\r", i, i == bitIndex, 
        //     (0 != (mode3Control & (1 << i))) );
        Concon_AddBit2BufferPos((0 != (mode3Control & (1 << i))), i, Mode3_DISP_COL-4);
    }
}

void __ISR(_SPI_1_VECTOR,SPI1_IPL)__SPI1_ISR(void)
{
    if (IFS1bits.SPI1TXIF) //If transmit buffer empty interrupt flag
    {

        if ( (1 == SPI1STATbits.SRMT) && (1 == SPI1STATbits.SPITBE) )
        {
            CONDISP_SS = IDLE;
            GASDISP_SS = IDLE;
            ACC_SS = IDLE;
            PostPilotSPI((ES_Event_t) {PSPI_XFER_DONE,0});
            IEC1bits.SPI1TXIE = 0;                // Disable transmit interrupt
        }
    }
    
    //Clear all flags
    IFS1bits.SPI1EIF = 0;
    IFS1bits.SPI1RXIF = 0;
    IFS1bits.SPI1TXIF = 0;
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

