/****************************************************************************
 Module
   Pilot_XBee.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

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
#include "Pilot_XBee.h"
#include <sys/attribs.h>

// relevant modules
#include "Pilot_CommComm.h"
#include "PilotService.h"
#include "PilotSPI.h"

// HALS
#include "../../HALs/PIC32PortHAL.h"
#include "../../HALS/CommStandards.h"
#include "../../HALs/dm_display.h"

/*----------------------------- Module Defines ----------------------------*/

// where are the pins
#define RX_PORT   _Port_B
#define RX_PIN    _Pin_11
#define RX_PIN_VAL  0b0011 // write this to U2RXR
#define TX_PORT   _Port_B
#define TX_PIN    _Pin_10
#define TX_PIN_REG  RPB10R // write to this to set value

// interrupt priorities
#define UART_IPC    6
#define UART_IPL    IPL6SOFT

// debugging
#define IGNORE_LOUDLY     // when message disappear, let us know!
//#define Announce_RX       /// tell us when we get messages
// #define DEBUG_fullATresponse // print all AT response
//#define Announce_FuelLevel    //  say fuel level at every recieve
// #define DEBUG_successRate   // how many packets are being recieved?
#define Display_activityBits // show on Concon display when values are moving

// activity bits locations
#define ACT_LED_ROW      1
#define ACT_LED_sentCol 15
#define ACT_LED_XBeeCol 13
#define ACT_LED_RecdCol 11
#define ACT_LED_AckdCol 9

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

//void __ISR(_UART_2_VECTOR, UART_IPL) UART_Interrupts(void);
uint8_t calculateChecksum(uint8_t* buffer, uint16_t length);
void startTransmission();
bool parseRXMessage();


/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match that of enum in header file
static Pilot_XBee_states_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// buffers and indexes
static uint8_t TX_buffer[MAX_MSG_LENGTH], RX_buffer[MAX_MSG_LENGTH];
static uint16_t TX_ind = 0;
static uint16_t RX_ind = 0;
static uint16_t TX_length = MAX_MSG_LENGTH+1;
static uint16_t RX_length = MAX_MSG_LENGTH+1;

static bool activeTransmission = false;
static bool alreadyReceiving = false;

// XBee address
static const uint16_t PILOT_addr = 0x218A; // this is US
// TUG we are paired to, inits to fake value
static uint16_t TUG_addr = dummyTUGaddr; 

// track frame success rate?
static uint32_t numFramesSent  = 0;
static uint32_t numFramesTXRd  = 0; // number of frames the TX reported seeign
static uint32_t numFramesReced = 0;
static uint32_t numFramesAcked = 0;

// internal stores of control values
static int8_t xControl, yControl, yawControl;
static uint8_t refuelControl, mode3Control;
static uint8_t fuelLevel; //how much fuel is left/
static bool sentOutOfFuel = false; // only send the event ONCE

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitPilotXBee(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  // ConfigUART();


  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = P_XBee_Init;
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
bool PostPilotXBee(ES_Event_t ThisEvent)
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
ES_Event_t RunPilotXBee(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case P_XBee_Init:        // If current state is initial Pseudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // enable global interrupts
        // __builtin_enable_interrupts();          // Enable global interrupts
        // printf("\n\n\rUART initialized, Interrupts on\n\r");

        // now put the machine into the actual initial state
        CurrentState = P_XBee_ready;
      }
      break;
    }

    // repeat state pattern as required for other states
    default:
    {
      ;
      break;
    }
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
Pilot_XBee_states_t QueryPilotXBee(void)
{
  return CurrentState;
}

/**
 * @brief configure UART subsystem (UART 2 for XBee)
 * 
 * Called from other state machine, need to enable GIE afterwards
 * 
 */
void ConfigUART(void)
{
    __builtin_disable_interrupts();         // Disable global interrupts
    
    INTCONbits.MVEC = 1;                    // Enable multi-vectored mode
    
    // Disable UART2 interrupts
    IEC1CLR = _IEC1_U2EIE_MASK  |           // Fault interrupt
              _IEC1_U2RXIE_MASK |           // Receiver interrupt
              _IEC1_U2TXIE_MASK;            // Transmitter interrupt
    
    U2MODE = 0;                             // Disable and reset Mode register
    U2STA = 0;                              // Reset Status register
    
    // Configure & assign RX input pin to UART2 module
    PortSetup_ConfigureDigitalInputs(RX_PORT,RX_PIN);
    U2RXR = RX_PIN_VAL;
    
    // Configure & assign TX output pin to UART2 module
    PortSetup_ConfigureDigitalOutputs(TX_PORT,TX_PIN);
    TX_PIN_REG = 0b0010;
    
    // Clear any existing UART2 interrupt flags
    IFS1CLR = _IFS1_U2EIF_MASK  |           // Fault interrupt
              _IFS1_U2RXIF_MASK |           // Receiver interrupt
              _IFS1_U2TXIF_MASK;            // Transmitter interrupt
    
    IPC9bits.U2IP = UART_IPC;               // Set interrupt priority level
    
    // Enable desired UART2 interrupts
    IEC1SET = _IEC1_U2RXIE_MASK;           // Receiver interrupt
              // _IEC1_U2TXIE_MASK;            // Transmitter interrupt
                // disable TX interrupts for now, only set when actively transmitting

    // Configure TX/RX interrupt frequency
    U2STAbits.UTXISEL = 0b01;               // interrupt when all characters send
    U2STAbits.URXISEL = 0b00;               // interrupt when at least 1 char
    
    // Configure UART2 Mode register
    U2MODESET = _U2MODE_RTSMD_MASK;         // U2RTS pin in Simplex mode
    
    // Configure UART2 Status register
    U2STASET = _U2STA_URXEN_MASK |          // Enable receiver
               _U2STA_UTXEN_MASK;           // Enable transmitter
    
    U2BRG = 129;                            // Set baud rate to 9600
    
    U2MODESET = _U2MODE_ON_MASK;            // Enable UART2 module
}

/***************************************************************************
 functions to create messages
 ***************************************************************************/

void constructATEcho() 
{
  // start stuffing data into value
  if (!activeTransmission) 
  {
    TX_buffer[startDelimiter_Posn] = XBee_START_DELIMITER;
    TX_buffer[lengthMSB_Posn]     = 0x00;
    TX_buffer[lengthLSB_Posn]     = 0x04;
    TX_buffer[APIID_Posn]         = ATcommandID;
    TX_buffer[FrameID_Posn]       = 0x52; // this is arbitrary!
    TX_buffer[ATcommand1_Posn]    = 'M';
    TX_buffer[ATcommand2_Posn]    = 'Y';

    TX_length = 7;
    // calculate checksum
    uint8_t checksum = calculateChecksum(TX_buffer, TX_length);
    TX_buffer[ATcmdChecksum_Posn] = checksum;

    startTransmission();  // begin transmission, Star-date: test
  }
  else // tell us why it's not being made
  {
    printf("Already sending message!");
  }
}

// call this once to select a new address
void constructSetPairingReq(uint16_t TUGaddr) {
  TUG_addr = TUGaddr; // store which TUG to pair to
  // reset numFrames
  numFramesSent  = 0;
  numFramesTXRd  = 0;
  numFramesReced = 0;
  numFramesAcked = 0;
  constructPairingReq(); // call without arg, actually send it
}

// call this repeatedly
void constructPairingReq()
{
  if (!activeTransmission) //prevent collisions, can't start sending yet
  {
    TX_buffer[startDelimiter_Posn] = XBee_START_DELIMITER;
    TX_buffer[lengthMSB_Posn]     = STD_PACKET_LENGTH_XB >> 8;
    TX_buffer[lengthLSB_Posn]     = STD_PACKET_LENGTH_XB;
    TX_buffer[APIID_Posn]         = TXrequestID;
    TX_buffer[FrameID_Posn]       = 0x01; // arbitrary!
    TX_buffer[DestAddrMSB_Posn]   = TUG_addr >> 8;
    TX_buffer[DestAddrLSB_Posn]   = TUG_addr;
    TX_buffer[options_Posn]       = 0x00; // don't change anything

    TX_buffer[msgID_Posn]         = RequestToPairID;
    TX_buffer[TUGaddrMSB_Posn]    = TUG_addr >> 8;
    TX_buffer[TUGaddrLSB_Posn]    = TUG_addr;
    TX_buffer[PILOTaddrMSB_Posn]  = PILOT_addr >> 8;
    TX_buffer[PILOTaddrLSB_Posn]  = PILOT_addr;
    TX_buffer[pairingACK_Posn]    = PairingACK_toTUG;
    TX_length = STD_PACKET_LENGTH_int; // how many bytes to send?
    uint8_t checksum = calculateChecksum(TX_buffer, TX_length);
    TX_buffer[checkSum_Posn]      = checksum;
    numFramesSent++; // track how many we've asked for
    startTransmission(); // send it!
  }
}


void constructControlFrame()
{
  if (!activeTransmission)
  {
    TX_buffer[startDelimiter_Posn] = XBee_START_DELIMITER;
    TX_buffer[lengthMSB_Posn]     = STD_PACKET_LENGTH_XB >> 8;
    TX_buffer[lengthLSB_Posn]     = STD_PACKET_LENGTH_XB;
    TX_buffer[APIID_Posn]         = TXrequestID;
    TX_buffer[FrameID_Posn]       = 0x01; // arbitrary!
    TX_buffer[DestAddrMSB_Posn]   = TUG_addr >> 8;
    TX_buffer[DestAddrLSB_Posn]   = TUG_addr;
    TX_buffer[options_Posn]       = 0x00; // don't change anything
    TX_buffer[msgID_Posn]         = controlFrameID;

    TX_buffer[xControl_Posn]      = xControl;
    TX_buffer[yControl_Posn]      = yControl;
    TX_buffer[yawControl_Posn]    = yawControl;
    TX_buffer[refuelControl_Posn] = refuelControl;
    TX_buffer[mode3Control_Posn]  = mode3Control;
    TX_length = STD_PACKET_LENGTH_int; // how many bytes to send?
    uint8_t checksum = calculateChecksum(TX_buffer, TX_length);
    TX_buffer[checkSum_Posn]      = checksum;
    numFramesSent++; // track how many we've asked for
    startTransmission(); // send it!

  }
}

void setControlValues(int8_t newXControl, int8_t newYControl,
                      int8_t newYawControl)
{
  xControl   = newXControl;
  yControl   = newYControl;
  yawControl = newYawControl;
}

void doTheRefuel()
{
  refuelControl = Refuel_True;
}

uint8_t getFuelLevel()
{
  return fuelLevel; // pass it out
}

void updateActivityLEDs(void) {
  #ifdef Display_activityBits
    Concon_AddBit2BufferPos(numFramesSent  % 2, ACT_LED_ROW, ACT_LED_sentCol );
    Concon_AddBit2BufferPos(numFramesTXRd  % 2, ACT_LED_ROW, ACT_LED_XBeeCol );
    Concon_AddBit2BufferPos(numFramesReced % 2, ACT_LED_ROW, ACT_LED_RecdCol );
    Concon_AddBit2BufferPos(numFramesAcked % 2, ACT_LED_ROW, ACT_LED_AckdCol );
  #endif /* Display_activityBits */
}

void setMode3Value(uint8_t newMode3)
{
  mode3Control = newMode3; // just a setter
}


/***************************************************************************
 private functions
 ***************************************************************************/

uint8_t calculateChecksum(uint8_t* buffer, uint16_t length)
{
  uint8_t sumVal = 0;
  for (uint16_t indLenght = (lengthLSB_Posn+1); indLenght < length; indLenght++)
  {
    sumVal += buffer[indLenght];
  }
  return (0xFF-sumVal); // pass as it should appear in message
}

void startTransmission()
{
  TX_ind = 0; // reset to the start
  U2TXREG = TX_buffer[TX_ind]; // first byte in
  activeTransmission = true;
  IEC1SET = _IEC1_U2TXIE_MASK; // turn on TX interrupts for now

  #ifdef DEBUG_successRate
    printf("Packets sent: %d, XB ack'd: %d, received: %d, ack'd: %d\n\r", 
            numFramesSent, numFramesTXRd, numFramesReced, numFramesAcked);
  #endif /* DEBUG_successRate */

  #ifdef Display_activityBits
    PostPilotSPI((ES_Event_t) { PSPI_Activity_Update, 0});
  #endif /* Display_activityBits */

}

void __ISR(_UART_2_VECTOR, UART_IPL) UART_Interrupts(void)
{
  if (IFS1bits.U2TXIF)
  {
    IFS1CLR = _IFS1_U2TXIF_MASK; // clear TX flag
    if ((activeTransmission) && (TX_ind < TX_length)) // not done sending yet
    {
      U2TXREG = TX_buffer[++TX_ind]; // send next byte
    }
    else //done sending data!
    {
      TX_ind = 0;
      activeTransmission = false; // no longer sending
      IEC1CLR = _IEC1_U2TXIE_MASK; // disable TX interrupts

    }
  }
  if (IFS1bits.U2RXIF)
  {
    IFS1CLR = _IFS1_U2RXIF_MASK; // clear RX flag

    static uint8_t newByte; // init static for speed
    newByte = U2RXREG; // read value in
    if (alreadyReceiving) // already in receiving state
    {
      RX_buffer[++RX_ind] = newByte;

      // check if we know length now
      if (RX_ind == lengthLSB_Posn) 
      {
        RX_length = RX_buffer[lengthMSB_Posn] << 8 | RX_buffer[lengthLSB_Posn];
        RX_length += 3; // 3 starting bytes (delimiter, 2x length) and 1 checksum
      }
      
      //did we finish receiving?
      if (RX_ind == RX_length)  
      {
        //message is done!
        parseRXMessage(); //parse it, do stuff
        RX_ind = 0; //reset
        alreadyReceiving = false; // no longer in middle of receiving
      }
    }
    else // not already receiving
    {
      if (XBee_START_DELIMITER == newByte) 
      {
        alreadyReceiving = true; //started a receive!
        RX_ind = 0;
        RX_length = MAX_MSG_LENGTH+1; // longer than possible!
        RX_buffer[RX_ind] = newByte; // store start delimiter!

      }
      else // not start delimiter, junk data?
      {
        printf("UART RX outside message: %x\n\r", newByte);
      }
    }
  }
}

bool parseRXMessage()
{
  uint8_t checksum = calculateChecksum(RX_buffer, RX_length);
  if (checksum == RX_buffer[RX_length]) {
    // checksums match, the message should be good!
    switch (RX_buffer[APIID_Posn]) // which API token do we have?
    {
      case ATresponseID: // AT command response
      {
        #ifdef DEBUG_fullATresponse
        printf("RX'd AT Command Response:\n\r\t");
        for (uint16_t ind = 0; ind < RX_length; ind++)
        {
          printf("%x ", RX_buffer[ind]); // write each value to screen
        }
        printf("\n\r"); // end the line
        #endif /* DEBUG_fullATresponse */
        
        printf("XBee ID: 0x%x%x\n\r", RX_buffer[8], RX_buffer[9]);
        break;
      }

      case TXstatusID: // status of TX packet
      {
        #ifdef Announce_RX
          printf("Got TX Status \n\r");
        #endif /* Announce_RX */
        numFramesTXRd++; // we saw it on the TX response
        //TODO: check frame ID?


        if (0 == RX_buffer[TXstatus_Posn]) // no errors!
        {
          numFramesReced++; // count how many ack's we got
        }
        break;
      }
      
      case RXpacketID:
      {
        #ifdef Announce_RX
          printf("Got RX Packet");
        #endif /* Announce_RX */
        // where did the message come from?
        uint16_t sourceAddr = RX_buffer[SourceAddrMSB_Posn] << 8 |
                              RX_buffer[SourceAddrLSB_Posn];
        if (TUG_addr == sourceAddr) // this message is from our paired TUG
        {
          switch (RX_buffer[msgID_Posn]) // which msg type do we have?
          {
            case PairingAckID:
            {
              uint16_t msgTUGAddr = RX_buffer[TUGaddrMSB_Posn] << 8 |
                                    RX_buffer[TUGaddrLSB_Posn];
              if ((TUG_addr == sourceAddr) && (TUG_addr == msgTUGAddr)) 
              {
                // all address match, should be a good message!
                // let pairing SM know
                PostPilotCommComm((ES_Event_t) {ES_PairingAck, 0});
                #ifdef Announce_RX
                  printf(", pairing ACK (from %x)", TUG_addr);
                #endif /* Announce_RX */
              }
              break;
            }

            case statusFrameID:
            {
              PostPilotCommComm((ES_Event_t) {ES_StatusPacketRecd, 0});
              numFramesAcked++; // track how many receives we get
              #ifdef Announce_RX
                printf(", status from (%x)", TUG_addr);
              #endif /* Announce_RX */
              
              fuelLevel = RX_buffer[fuelLevel_Posn]; // read data out
              if (0 != fuelLevel) 
              {
                // we have fuel, can't refuel
                refuelControl = false;
                sentOutOfFuel = false; // re-eanble sending event!
              }
              else if (false == sentOutOfFuel) // out of fuel!
              {
                PostPilotService((ES_Event_t) {PS_REFUEL, 0});
                sentOutOfFuel = true; //
              }

              // tell display to update
              PostPilotService
                      ((ES_Event_t) {PS_FUEL_UPDATE, 0});
              
              #ifdef Announce_FuelLevel
                printf("Fuel Level: %d\n\r", fuelLevel);
              #endif /* Announce_FuelLevel */
              break;
            }
            
            default:
              break;
          }
          #ifdef Announce_RX
            printf("\n\r"); // add newline at total end
          #endif /*Announce_RX */

        }
        break;
      }
      
      default:
        break;
    }
  }
  else //bad checksum
  {
    #ifdef IGNORE_LOUDLY
      printf("Bad checksum on RX!\n\r");
    #endif /* IGNORE_LOUDLY */
  }
}