/**
 * @file CommStandards.h
 * @author Josh DeWitt (jndewitt@stanford.edu)
 * @brief standards and byte locations for the 
 *      ME218C 2022 Communication Protocol
 * @version 0.1
 * @date 2022-05-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CommStandards_H
#define CommStandards_H

// message ID - internal to our communication protocol
enum messageIDs {
    controlFrameID  = 0x01,
    statusFrameID   = 0x02,
    RequestToPairID = 0x03,
    PairingAckID    = 0x04,
};

// API IDs, identifies which type of frame to use
enum XBeeAPIIDs {
    TXrequestID = 0x01,
    ATcommandID = 0x08,
    RXpacketID = 0x81,
    ATresponseID = 0x88,
    TXstatusID = 0x89,
};

// timing definitions
#define PacketRequestTimeout    200     // [ms] packet frequency = 5Hz
#define CommunicationsTimeout   3000    // [ms] loss of comms

// Pairing constants
#define PairingACK_toTUG    0xAA // 1010 1010 lol
#define PairingACK_toPILOT  0x55 // 0101 0101

//how to start a message
#define XBee_START_DELIMITER    0x7E 

// max reasonable length of a message
#define MAX_MSG_LENGTH      128 // can do 15+100+1 for 64 bit send
                                // this gives us some buffer over that

// standard packet length 
#define STD_PACKET_LENGTH_XB   0x0B // to but into MSB of packet itself
#define STD_PACKET_LENGTH_int  14 // number of bytes for TX/RX_ind

#define Refuel_True            0x01 //what is consider "true" for refuel

// byte locations common between all types
enum msgFrame_Universal {
    startDelimiter_Posn = 0,
    lengthMSB_Posn,
    lengthLSB_Posn,
    APIID_Posn = 3,
    options_Posn = 7,
    msgID_Posn = 8,
    checkSum_Posn = 14,
};

// XBee locations for sending
// applies to `TXrequestID`
enum msgFrame_Sending {
    FrameID_Posn = 4,
    DestAddrMSB_Posn,
    DestAddrLSB_Posn,
    OptionsSending_Posn,
};

// XBee locations for receiving
// applies to `RXpacketID`
enum msgFrame_Receiving {
    SourceAddrMSB_Posn = 4,
    SourceAddrLSB_Posn,
    RSSIvalue_Posn,
};

// XBee locations for TX status
// applies to `TXstatusID`
enum msgFrame_TXstatus {
    TXstatus_Posn = 5,
};

//XBee locations for AT commands
// appliers to `ATcommandID`
// use immediately below for no params
enum msgFrame_ATcmds {
    ATcommand1_Posn = 5,
    ATcommand2_Posn,
    ATcmdChecksum_Posn,
};

//used when sending commands with params
    // probably not used tbh
enum msgFrame_ATcmdsParams {
    ATParam1_Posn = 7,
    ATParam2_Posn,
    ATParam3_Posn,
    ATParam4_Posn,
    ATParamChecksum_Posn,
};

// Comm Protocol data frame locations

enum dataFrame_Pairing {
    TUGaddrMSB_Posn = 9,
    TUGaddrLSB_Posn,
    PILOTaddrMSB_Posn,
    PILOTaddrLSB_Posn,
    pairingACK_Posn,
};

enum dataFrame_Control {
    xControl_Posn = 9,
    yControl_Posn,
    yawControl_Posn,
    refuelControl_Posn,
    mode3Control_Posn,
};

enum dataFrame_status {
    fuelLevel_Posn = 9,
    statUnimp1_Posn,
    statUnimp2_Posn,
    statUnimp3_Posn,
    statUnimp4_Posn,
};

// // TUG addresses
// //not in team order b/c enums are specific
// typedef enum {
//   Team1_TUG  = 0x2017,
//   Team4_TUG  = 0x2085,
//   NoTeam_TUG = 0x2169,
//   Team2_TUG  = 0x2184,
//   Team0_TUG  = 0x2187,
//   Team5_TUG  = 0x2185,
//   Team3_TUG  = 0x2188,
// } TUG_addr_t;

#define Num_TUGs    7 // total number of TUG pairing options
extern const uint16_t TUG_Addresses[Num_TUGs];
#define ourTUGaddr  0x2187 // our tug address
#define dummyTUGaddr 0x2169 // won't really pair


#endif /* CommStandards_H */