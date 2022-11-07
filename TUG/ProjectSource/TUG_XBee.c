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
#include "TUG_XBee.h"
#include <sys/attribs.h>

// relevant modules
#include "TUG_CommComm.h"
#include "PumpService.h"

// HALS
#include "../../HALs/PIC32PortHAL.h"
#include "../../HALS/CommStandards.h"
    
/*----------------------------- Module Defines ----------------------------*/

// where are the pins
#define RX_PORT   _Port_B
#define RX_PIN    _Pin_11
#define RX_PIN_VAL  0b0011 // write this to U2RXR
#define TX_PORT   _Port_B
#define TX_PIN    _Pin_10
#define TX_PIN_REG  RPB10R // write to this to set value

// mode 3 bit positions
#define INF_Fuel_Posn   0 // infinite fuel is at MSB

// interrupt priorities
#define UART_IPC    6
#define UART_IPL    IPL6SOFT

// debugging
#define IGNORE_LOUDLY     // when message disappear, let us know!
//#define Announce_RX       /// tell us when we get messages
// #define DEBUG_fullATresponse // print all AT response
//#define Announce_Ctrls    // say what new control values are

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

uint8_t calculateChecksum(uint8_t* buffer, uint16_t length);
void startTransmission();
bool parseRXMessage();
void parseMode3();


/*---------------------------- Module Variables ---------------------------*/

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
static uint16_t PILOT_addr = dummyTUGaddr; // Pilot we are paired to, inits to fake value
static const uint16_t TUG_addr = ourTUGaddr; // this is US

// track frame success rate?
static uint32_t numFramesSent  = 0;
static uint32_t numFramesReced = 0;
static uint32_t numFramesAcked = 0;

// internal stores of control values
static int8_t xControl, yControl, yawControl;
static uint8_t refuelControl, mode3Control;

// mode3 eventCheckers
static uint8_t lastmode3Control = 0;

/*------------------------------ Module Code ------------------------------*/


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

// call this repeatedly
void constructPairingAck()
{
  if (!activeTransmission) //prevent collisions, can't start sending yet
  {
    TX_buffer[startDelimiter_Posn] = XBee_START_DELIMITER;
    TX_buffer[lengthMSB_Posn]     = STD_PACKET_LENGTH_XB >> 8;
    TX_buffer[lengthLSB_Posn]     = STD_PACKET_LENGTH_XB;
    TX_buffer[APIID_Posn]         = TXrequestID;
    TX_buffer[FrameID_Posn]       = 0x01; // arbitrary!
    TX_buffer[DestAddrMSB_Posn]   = PILOT_addr >> 8;
    TX_buffer[DestAddrLSB_Posn]   = PILOT_addr;
    TX_buffer[options_Posn]       = 0x00; // don't change anything

    TX_buffer[msgID_Posn]         = PairingAckID;
    TX_buffer[TUGaddrMSB_Posn]    = TUG_addr >> 8;
    TX_buffer[TUGaddrLSB_Posn]    = TUG_addr;
    TX_buffer[PILOTaddrMSB_Posn]  = PILOT_addr >> 8;
    TX_buffer[PILOTaddrLSB_Posn]  = PILOT_addr;
    TX_buffer[pairingACK_Posn]    = PairingACK_toPILOT;
    TX_length = STD_PACKET_LENGTH_int; // how many bytes to send?
    uint8_t checksum = calculateChecksum(TX_buffer, TX_length);
    TX_buffer[checkSum_Posn]      = checksum;
    startTransmission(); // send it!
  }
}


void constructStatusFrame()
{
  if (!activeTransmission)
  {
    TX_buffer[startDelimiter_Posn] = XBee_START_DELIMITER;
    TX_buffer[lengthMSB_Posn]     = STD_PACKET_LENGTH_XB >> 8;
    TX_buffer[lengthLSB_Posn]     = STD_PACKET_LENGTH_XB;
    TX_buffer[APIID_Posn]         = TXrequestID;
    TX_buffer[FrameID_Posn]       = 0x01; // arbitrary!
    TX_buffer[DestAddrMSB_Posn]   = PILOT_addr >> 8;
    TX_buffer[DestAddrLSB_Posn]   = PILOT_addr;
    TX_buffer[options_Posn]       = 0x00; // don't change anything
    TX_buffer[msgID_Posn]         = statusFrameID;

    TX_buffer[fuelLevel_Posn]     = getRemainingFuel();
    TX_buffer[statUnimp1_Posn]    = 0x00;
    TX_buffer[statUnimp2_Posn]    = 0x00;
    TX_buffer[statUnimp3_Posn]    = 0x00;
    TX_buffer[statUnimp4_Posn]    = 0x00;
    TX_length = STD_PACKET_LENGTH_int; // how many bytes to send?
    uint8_t checksum = calculateChecksum(TX_buffer, TX_length);
    TX_buffer[checkSum_Posn]      = checksum;
    startTransmission(); // send it!
    numFramesSent++; // track how many we've asked for

  }
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
//    printf("\t ind: %d, val: %x\n\r", indLenght, buffer[indLenght]);
  }
//  printf("sumval: %x ", sumVal);
  return (0xFF-sumVal); // pass as it should appear in message
}

void startTransmission()
{
  TX_ind = 0; // reset to the start
  U2TXREG = TX_buffer[TX_ind]; // first byte in
  activeTransmission = true;
  IEC1SET = _IEC1_U2TXIE_MASK; // turn on TX interrupts for now
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
        switch (RX_buffer[msgID_Posn]) // which msg type do we have?
        {
          case RequestToPairID:
          {
            uint16_t msgPILOTaddr = RX_buffer[PILOTaddrMSB_Posn] << 8 |
                                    RX_buffer[PILOTaddrLSB_Posn];
            if (msgPILOTaddr == sourceAddr) 
            {
              // addresses match, message made correctly
              // let pairing SM know
              PostTUGCommComm((ES_Event_t) {ES_PairingRequest, 0});
              PILOT_addr = sourceAddr; // store paired PILOT address
              #ifdef Announce_RX
                printf(", Request to Pair (from %x)", PILOT_addr);
              #endif /* Announce_RX */
            }
            break;
          }

          case controlFrameID:
          {
            if (PILOT_addr == sourceAddr) // this message is from our paired PILOT
            {
              PostTUGCommComm((ES_Event_t) {ES_ControlPacketRecd, 0});
              //TODO: tell motors we have new data
              // parse control data
              xControl      = (int8_t)  RX_buffer[xControl_Posn];
              yControl      = (int8_t)  RX_buffer[yControl_Posn];
              yawControl    = (int8_t)  RX_buffer[yawControl_Posn];
              refuelControl = (uint8_t) RX_buffer[refuelControl_Posn];
              mode3Control  = (uint8_t) RX_buffer[mode3Control_Posn];
              UpdateThrustVectors(xControl, yControl, yawControl);
              PostPumpService((ES_Event_t) {EV_THRUST_UPDATE, 0});

              if (refuelControl)
              {
                // got refuel data, post refuel event!
                PostPumpService((ES_Event_t) {ES_RefuelCmd, 0});
              }

              parseMode3(); // update aux byte stuffs

              #ifdef Announce_Ctrls
                printf("\t New Controls: X:%d\t Y:%d\t Yaw:%d\t Refuel:%d\t Mode3:%d\t \n\r",
                        xControl, yControl, yawControl, refuelControl, mode3Control);
              #endif /* Announce_Ctrls */

              numFramesAcked++; // track how many receives we get
              #ifdef Announce_RX
                printf(", control data");
              #endif /* Announce_RX */
            }
            break;
          }
          
          default:
            break;
        }
        #ifdef Announce_RX
          printf("\n\r"); // add newline at total end
        #endif /*Announce_RX */

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
      printf("Rec: %x, calc: %x\n\r", RX_buffer[RX_length-2], checksum);
    #endif /* IGNORE_LOUDLY */
  }
}

void parseMode3()
{
  // printf("Mode3: %x\n\r", mode3Control);
  // check for constant events
  // mode 3 MSB, infinite fuel!!
  if (0 != (mode3Control & (1 << INF_Fuel_Posn)))
  {
    PostPumpService((ES_Event_t) {ES_RefuelCmd, 1}); // refuel!
  }

  // check for changes
  uint8_t m3Changes = lastmode3Control ^ mode3Control;

  if (0 != (m3Changes & (1 << INF_Fuel_Posn))) 
  {
    printf("Infinite Fuel Toggled: %d\n\r", 
           (0!= (mode3Control & (1 << INF_Fuel_Posn))));
  }

  lastmode3Control = mode3Control; // update for next time
}