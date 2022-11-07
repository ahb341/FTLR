/****************************************************************************
 Module
     PIC32_SPI_HAL.c
 Description
     Source file for the PIC32 SPI Hardware Abstraction Layer used in ME218
 Notes
     This is the prototype. Students will re-create this functionality
 History
 When           Who     What/Why
 -------------- ---     --------
  10/03/21 12:32 jec    started coding
*****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <xc.h>
#include <stdbool.h> 
#include <math.h>
#include "PIC32_SPI_HAL.h"

/*--------------------------- External Variables --------------------------*/

/*----------------------------- Module Defines ----------------------------*/
// this is based on a 13 bit (max=8191) BRG register and 20MHz (50ns) PBCLK
#define MIN_SPI_PERIOD  (50)                        //time in ns
#define MAX_SPI_PERIOD  ((8191+1)*2*MIN_SPI_PERIOD) //max time if func of min
#define MAX_PBC_FREQ    (20e6)                      //frequncy of PB clock
#define MAP_SS1 0b0011
#define MAP_SS2 0b0100
#define MAP_SDO1 0b0011
#define MAP_SDO2 0b0100

/*------------------------------ Module Types -----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
static void selectModuleRegisters(SPI_Module_t WhichModule);
static bool isSPI_ModuleLegal( SPI_Module_t WhichModule);
static bool isSS_OutputPinLegal(SPI_Module_t WhichModule, 
                                SPI_PinMap_t WhichPin);
static bool isSDOPinLegal(SPI_PinMap_t WhichPin);
static bool isSDIPinLegal(SPI_Module_t WhichModule,
                          SPI_PinMap_t WhichPin);

/*---------------------------- Module Variables ---------------------------*/
  // these will allow us to reference both SPI1 & SPI2 through these pointers
static volatile __SPI1CONbits_t * pSPICON;   
static volatile __SPI1CON2bits_t * pSPICON2;
static volatile uint32_t * pSPIBRG;
static volatile uint32_t * pSPIBUF;
static volatile __SPI1STATbits_t * pSPISTAT;
//have we started a transmission on a SPI bus?
static bool SPI1_hasWritten, SPI2_hasWritten; 

// these are the output mapping registers indexed by the SPI_PinMap_t value
static volatile uint32_t * const outputMapRegisters[] = { &RPA0R, &RPA1R, 
                      &RPA2R, &RPA3R, &RPA4R, 
                      &RPB0R, &RPB1R, &RPB2R, &RPB3R, &RPB4R, &RPB5R, 
                      &RPB6R, &RPB7R, &RPB8R, &RPB9R, &RPB10R, &RPB11R, &RPB12R,
                      &RPB13R, &RPB14R, &RPB15R 
};

// these are the TRISxSET registers indexed by the SPI_PinMap_t value
static volatile uint32_t * const setTRISRegisters[] = { &TRISASET, &TRISASET,
            &TRISASET, &TRISASET, &TRISASET,
            &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET,
            &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET, 
            &TRISBSET, &TRISBSET, &TRISBSET, &TRISBSET
};

// these are the TRISxCLR registers indexed by the SPI_PinMap_t value
static volatile uint32_t * const clrTRISRegisters[] = { &TRISACLR, &TRISACLR,
            &TRISACLR, &TRISACLR, &TRISACLR,
            &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR,
            &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR, 
            &TRISBCLR, &TRISBCLR, &TRISBCLR, &TRISBCLR
};

// these are the ANSELxCLR registers indexed by the SPI_PinMap_t value
static volatile uint32_t * const clrANSELRegisters[] = { &ANSELACLR, &ANSELACLR,
         &ANSELACLR, &ANSELACLR, &ANSELACLR,
         &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR,
         &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, 
         &ANSELBCLR, &ANSELBCLR, &ANSELBCLR, &ANSELBCLR
};

// these are the bit positions indexed by the SPI_PinMap_t value
static uint32_t const mapPinMap2BitPosn[] = { 1<<0, 1<<1,
         1<<2, 1<<3, 1<<4,
         1<<0, 1<<1, 1<<2, 1<<3, 1<<4, 1<<5,
         1<<6, 1<<7, 1<<8, 1<<9, 1<<10, 1<<11, 
         1<<12, 1<<13, 1<<14, 1<<15
};

// these are the INT pin mapping constants indexed by the SPI_PinMap_t value
static uint32_t const mapPinMap2INTConst[] = { 0b0000/*RA0*/, 0b0000/*RA1*/,
         0b0000/*RA2*/, 0b0000/*RA3*/, 0b0010/*RA4*/,
         0b0010/*RB0*/,0b0010/*RB1*/, 0b0100/*RB2*/, 0b0001/*RB3*/, 
         0b0010/*RB4*/, 0b0001/*RB5*/, 0b0001/*RB6*/, 0b0100/*RB7*/, 
         0b0100/*RB8*/, 0b0100/*RB9*/, 0b0011/*RB10*/, 0b0011/*RB11*/, 
         0/*RB12*/, 0b0011/*RB13*/, 0b0001/*RB14*/, 0b0011/*RB15*/
};

static SPI_PinMap_t const LegalSSOutPins[][5] = {{ SPI_RPA0, SPI_RPB3, SPI_RPB4, 
                                             SPI_RPB7,SPI_RPB15 },
                                             { SPI_RPA3, SPI_RPB0, SPI_RPB9, 
                                             SPI_RPB10,SPI_RPB14 }
};

static SPI_PinMap_t const LegalSDOxPins[] = { SPI_NO_PIN, SPI_RPA1, SPI_RPA2, 
                                              SPI_RPA4, SPI_RPB1, SPI_RPB2, 
                                              SPI_RPB5, SPI_RPB6, SPI_RPB8, 
                                              SPI_RPB11, SPI_RPB13 
};

static SPI_PinMap_t const LegalSDIxPins[2][6] = {
  {SPI_NO_PIN, SPI_RPA1, SPI_RPB1, SPI_RPB5, SPI_RPB8,SPI_RPB11},
  {SPI_NO_PIN, SPI_RPA2, SPI_RPA4, SPI_RPB2, SPI_RPB6,SPI_RPB13}  
};

//how to write to the SDIxR registers
static uint32_t const SDIMapping[] = { 
  0b1111/*RA0 */, 0b0000/*RA1 */,
  0b0000/*RA2 */, 0b1111/*RA3 */, 0b0010/*RA4 */,
  0b1111/*RB0 */, 0b0010/*RB1 */, 0b0100/*RB2 */, 0b1111/*RB3*/, 
  0b1111/*RB4 */, 0b0001/*RB5 */, 0b0001/*RB6 */, 0b1111/*RB7*/, 
  0b0100/*RB8 */, 0b1111/*RB9 */, 0b1111/*RB10*/, 0b0011/*RB11*/, 
  0b1111/*RB12*/, 0b0011/*RB13*/, 0b1111/*RB14*/, 0b1111/*RB15*/,
  0b1111/*NO PIN*/
};

//how to write to the SSxR registers
static uint32_t const SSInMapping[] = { 
  0b0000/*RA0 */, 0b1111/*RA1 */,
  0b1111/*RA2 */, 0b0000/*RA3 */, 0b1111/*RA4 */,
  0b0010/*RB0 */, 0b1111/*RB1 */, 0b1111/*RB2 */, 0b0001/*RB3*/, 
  0b0010/*RB4 */, 0b1111/*RB5 */, 0b1111/*RB6 */, 0b0100/*RB7*/, 
  0b1111/*RB8 */, 0b0100/*RB9 */, 0b0011/*RB10*/, 0b1111/*RB11*/, 
  0b1111/*RB12*/, 0b1111/*RB13*/, 0b0001/*RB14*/, 0b0011/*RB15*/,
  0b1111/*NO PIN*/
};

/*------------------------------ Module Code ------------------------------*/


/****************************************************************************
 Function
    SPISetup_BasicConfig

 Description
   Should be the first function called when setting up an SPI module.
   1) Disables the selected SPI Module
   2) Configures the SPI clock to be based on PBCLK
   3) Disables the Framed mode
   4) Disables the Audio mode
   Further function calls from the SPI HAL will be necessary to complete 
   the module setup.
****************************************************************************/
bool SPISetup_BasicConfig(SPI_Module_t WhichModule)
{
  bool ReturnVal = true;
   
  // Make sure that we have a legal module specified
  if ( false == isSPI_ModuleLegal(WhichModule))
  {
      ReturnVal = false;
  }else // Legal module so set it up
  {
     selectModuleRegisters(WhichModule); 
  
    pSPICON->ON = 0;        // Disable the selected SPI Module
    pSPICON->MCLKSEL = 0;   // Configure the SPI clock to be based on PBCLK
    pSPICON->FRMEN = 0;     // Disable the Framed mode
    pSPICON2->AUDEN = 0;    // Disable the Audio mode

    SPI1_hasWritten = 0;    // neither SPI1 or SPI2 should have written
    SPI2_hasWritten = 0;    // at this point, so keep them both low
  }   
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_SetFollower

 Description
   Sets the selected SPI Module to Follower mode and configures the SPI CLK 
   pin as an input. NOTE: 
   1) Either this function or the SPISetup_SetLeader function 
   should be called immediately after the call to SPISetup_BasicConfig. 
   2) the PIC32 documentation refers to this mode as slave mode.
****************************************************************************/
bool SPISetup_SetFollower(SPI_Module_t WhichModule)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    pSPICON->MSTEN = 0;   //disable leader (master) mode
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_SetLeader

 Description
   Sets the selected SPI Module to leader mode, configures the SPI CLK 
   pin as an output, and sets the input sample phase. 
   NOTE: 1) Either this function or the SPISetup_SetFollower function should 
   be called immediately after the call to SPISetup_BasicConfig. 
   2) the PIC32 documentation refers to this mode as master mode.
****************************************************************************/
bool SPISetup_SetLeader(SPI_Module_t WhichModule, SPI_SamplePhase_t WhichPhase)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else if ((WhichPhase != SPI_SMP_END) && (WhichPhase != SPI_SMP_MID))
  {
    ReturnVal = false; //exit, illegal sample phase
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    pSPICON->MSTEN = 1;   //enable leader (master) mode
    pSPICON->SMP = WhichPhase; //this only works through a coincidence, but it works
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_SetBitTime

 Description
   Based on a 20MHz PBCLK, calculates and programs the SPIBRG register for the
   specified SPI module to achieve the requested bit time.
****************************************************************************/
bool SPISetup_SetBitTime(SPI_Module_t WhichModule, uint32_t SPI_ClkPeriodIn_ns)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else if ((SPI_ClkPeriodIn_ns < MIN_SPI_PERIOD) || (SPI_ClkPeriodIn_ns > MAX_SPI_PERIOD) )
  {
    ReturnVal = false; //exit, time is impossible
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    uint32_t Fspi = 1/(SPI_ClkPeriodIn_ns*1e-9); //find period

    uint32_t BRGy = ceil(MAX_PBC_FREQ/(2*Fspi)-1); //calculate divisor
    *(pSPIBRG) = BRGy; //write to BRG register

  }
  return ReturnVal;
}
/****************************************************************************
 Function
    SPISetup_MapSSInput

 Description
   Sets the designated pin to be the SS input if the selected SPI Module
   is configured in Follower mode. 
   Legal port pins for the SS1 input are:
   SPI_RPA0, SPI_RPB3, SPI_RPB4, SPI_RPB7,SPI_RPB15.
   Legal port pins for the SS2 input are:
   SPI_RPA3, SPI_RPB0, SPI_RPB9, SPI_RPB10,SPI_RPB14.
****************************************************************************/
bool SPISetup_MapSSInput(SPI_Module_t WhichModule, SPI_PinMap_t WhichPin)
{
  // not needed for ME218a Labs
  bool ReturnVal = false;

  // Make sure that we have a legal module specified & legal pin
  if ( (false == isSPI_ModuleLegal(WhichModule)) || 
       (false == isSS_OutputPinLegal(WhichModule, WhichPin)) )
  {
    ReturnVal = false;
  }else 
  { // Legal module  & pin so set try setting it up
    selectModuleRegisters(WhichModule);
    if (0 == pSPICON->MSTEN)  // configured in Follower mode?
    {
      if (SPI_NO_PIN == WhichPin)
      {
        pSPICON->SSEN = 0; // disable SS
      }else //there is an SS pin so map it
      {
        pSPICON->SSEN = 1; // enable SS
        // set the TRIS bit to make it an input
        *setTRISRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
        // clear the ANSEL bit to disable analog on the pin
        *clrANSELRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];

        if (SPI_SPI1 == WhichModule)
        {
          SS1R = SSInMapping[WhichPin]; // Map SS to chosen pin
        }else  
        {   // must be SPI2 so set up INT1
          SS2R = SSInMapping[WhichPin]; // Map SS to chosen pin
        }
      }
    }else // not in Leader mode
    {
      ReturnVal = false; // then we can't config an SS output
    }
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_MapSSOutput

 Description
   Sets the designated pin to be the SS output if the selected SPI Module
   is configured in Leader mode. Clears TRIS and ANSEL to make pin an output.
   Also configures INT4/INT1 to monitor for rising edges on the SS output pin.
   Legal port pins for the SS1 output are:
   SPI_NO_PIN, SPI_RPA0, SPI_RPB3, SPI_RPB4, SPI_RPB7,SPI_RPB15.
   Legal port pins for the SS2 output are:
   SPI_NO_PIN, SPI_RPA3, SPI_RPB0, SPI_RPB9, SPI_RPB10,SPI_RPB14.
****************************************************************************/
bool SPISetup_MapSSOutput(SPI_Module_t WhichModule, SPI_PinMap_t WhichPin)
{
  bool ReturnVal = true;
  
  // Make sure that we have a legal module specified & legal pin
  if ( (false == isSPI_ModuleLegal(WhichModule)) || 
       (false == isSS_OutputPinLegal(WhichModule, WhichPin)) )
  {
    ReturnVal = false;
  }else 
  { // Legal module  & pin so set try setting it up
    selectModuleRegisters(WhichModule);
    if (1 == pSPICON->MSTEN)  // configured in Leader mode?
    {
      if (SPI_NO_PIN == WhichPin)
      {
        pSPICON->MSSEN = 0; // disable SS
      }else //there is an SS pin so map it
      {
        pSPICON->MSSEN = 1; // enable SS
        // clear the TRIS bit to make it an output
        *clrTRISRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
        // clear the ANSEL bit to disable analog on the pin
        *clrANSELRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
            
        if (SPI_SPI1 == WhichModule)
        {
          *outputMapRegisters[WhichPin] = MAP_SS1; // Map SS to chosen pin
          // set up to use INT4 to capture the rising edge of SS
          INTCONbits.INT4EP = 1;            // set for rising edge sensitivity
          IFS0CLR = _IFS0_INT4IF_MASK;      // clear any pending flag
          INT4R = mapPinMap2INTConst[WhichPin];  // map INT4 to SS as well
        }else  
        {   // must be SPI2 so set up INT1
          *outputMapRegisters[WhichPin] = MAP_SS2; // Map SS to chosen pin
          // set up to use INT1 to capture the rising edge of SS
          INTCONbits.INT1EP = 1;            // set for rising edge sensitivity
          IFS0CLR = _IFS0_INT1IF_MASK;      // clear any pending flag
          INT1R = mapPinMap2INTConst[WhichPin];  // map INT1 to SS as well
        }
      }
    }else // not in Leader mode
    {
      ReturnVal = false; // then we can't config an SS output
    }
  }
  return ReturnVal;
}
/****************************************************************************
 Function
    SPISetup_MapSDInput

 Description
   Sets the designated pin to be the SD input.
   Legal port pins for the SDI1 input are:
   SPI_NO_PIN, SPI_RPA1, SPI_RPB1, SPI_RPB5, SPI_RPB8,SPI_RPB11.   
   Legal port pins for the SDI2 input are:
   SPI_NO_PIN, SPI_RPA2, SPI_RPA4, SPI_RPB2, SPI_RPB6,SPI_RPB13.   
****************************************************************************/
bool SPISetup_MapSDInput(SPI_Module_t WhichModule, SPI_PinMap_t WhichPin)
{
  // not needed for ME218a Labs
  bool ReturnVal = true;
  // Your Code goes here :-)
  // Make sure that we have a legal module specified & legal pin
  if ( (false == isSPI_ModuleLegal(WhichModule)) || 
       (false == isSDIPinLegal(WhichModule, WhichPin)) )
  {
    ReturnVal = false;
  }else 
  { // Legal module  & pin so set try setting it up
    selectModuleRegisters(WhichModule);
    
    if (SPI_NO_PIN == WhichPin)
    {
      pSPICON->DISSDI = 1; // disable SDI
    }else //there is an SS pin so map it
    {
      pSPICON->DISSDI = 0; // enable SDI
      // set the TRIS bit to make it an input
      *setTRISRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
      // clear the ANSEL bit to disable analog on the pin
      *clrANSELRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
          
      if (SPI_SPI1 == WhichModule)
      {
        SDI1R = SDIMapping[WhichPin]; // Map SDO to chosen pin
      }else  
      {   // must be SPI2 so set up INT1
        SDI2R = SDIMapping[WhichPin]; // Map SDO to chosen pin
      }
    }
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_MapSDOutput

 Description
   Sets the designated pin to be the SD output. 
****************************************************************************/
bool SPISetup_MapSDOutput(SPI_Module_t WhichModule, SPI_PinMap_t WhichPin)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  // Make sure that we have a legal module specified & legal pin
  if ( (false == isSPI_ModuleLegal(WhichModule)) || 
       (false == isSDOPinLegal(WhichPin)) )
  {
    ReturnVal = false;
  }else 
  { // Legal module  & pin so set try setting it up
    selectModuleRegisters(WhichModule);
    
    if (SPI_NO_PIN == WhichPin)
    {
      pSPICON->DISSDO = 1; // disable SDO
    }else //there is an SS pin so map it
    {
      pSPICON->DISSDO = 0; // enable SDO
      // clear the TRIS bit to make it an output
      *clrTRISRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
      // clear the ANSEL bit to disable analog on the pin
      *clrANSELRegisters[WhichPin] = mapPinMap2BitPosn[WhichPin];
          
      if (SPI_SPI1 == WhichModule)
      {
        *outputMapRegisters[WhichPin] = MAP_SDO1; // Map SDO to chosen pin
      }else  
      {   // must be SPI2 so set up INT1
        *outputMapRegisters[WhichPin] = MAP_SDO2; // Map SDO to chosen pin
      }
    }
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_SetClockIdleState

 Description
   Sets the idle state of the SPI clock. 
****************************************************************************/
bool SPISetup_SetClockIdleState(SPI_Module_t WhichModule, 
                                SPI_Clock_t WhichState)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else if ((WhichState != SPI_CLK_HI) && (WhichState != SPI_CLK_LO))
  {
    ReturnVal = false; //exit, illegal clock idle
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    bool idles; //init variable for hi/lo clock
    if(WhichState == SPI_CLK_HI)
    {
      idles = 1; //high idle
    }
    else if(WhichState == SPI_CLK_LO)
    {
      idles = 0; //low idle
    }
    pSPICON->CKP = idles; //set clock polarity
  }
  return ReturnVal;
  
}
/****************************************************************************
 Function
    SPISetup_SetActiveEdge

 Description
   Sets the active edge of the SPI clock. 
****************************************************************************/
bool SPISetup_SetActiveEdge(SPI_Module_t WhichModule, 
                            SPI_ActiveEdge_t WhichEdge)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else if (!(WhichEdge == SPI_FIRST_EDGE) && !(WhichEdge == SPI_SECOND_EDGE))
  {
    ReturnVal = false; //exit, illegal clock idle
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    bool edges; //init variable for clock edge
    if(WhichEdge == SPI_FIRST_EDGE)
    {
      edges = 1; //latches on first edge, changes on second (active->idle)
    }
    else if(WhichEdge == SPI_SECOND_EDGE)
    {
      edges = 0; //latches on second edge, changes on first (idle->active)
    }
    pSPICON->CKE = edges; //set clock polarity
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_SetXferWidth

 Description
   Sets the width of the transfers that the SPI module will perform. 
****************************************************************************/
bool SPISetup_SetXferWidth(SPI_Module_t WhichModule, 
                            SPI_XferWidth_t DataWidth)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else if (!(DataWidth == SPI_8BIT) &&
           !(DataWidth == SPI_16BIT) &&
           !(DataWidth == SPI_32BIT))
  {
    ReturnVal = false; //exit, illegal data width
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    bool mode16; //init variable for 16 bit
    bool mode32; //init variable for 32 bit
    if(DataWidth == SPI_8BIT)
    {
      mode16 = 0; mode32 = 0;  // 8bit mode
    }
    else if(DataWidth == SPI_16BIT)
    {
      mode16 = 1; mode32 = 0;  // 16bit mode
    }
    else if(DataWidth == SPI_32BIT)
    {
      mode16 = 0; mode32 = 1;  // 8bit mode
    }
    pSPICON->MODE32 = mode32;
    pSPICON->MODE16 = mode16; //set bits in register
  }
  return ReturnVal;  
}

/****************************************************************************
 Function
    SPISetEnhancedBuffer

 Description
   Enables/disables the enhanced buffer on a module based on the second param 
****************************************************************************/
bool SPISetEnhancedBuffer(SPI_Module_t WhichModule, bool IsEnhanced)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    pSPICON->ENHBUF = IsEnhanced; //switch enhanced buffer
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPISetup_DisableSPI

 Description
   Disables the selected SPI Module
****************************************************************************/
bool SPISetup_DisableSPI(SPI_Module_t WhichModule)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    pSPICON->ON = 0;   //disable SPI
  }
  return ReturnVal;
  
}
/****************************************************************************
 Function
    SPISetup_EnableSPI

 Description
   Enables the selected SPI Module
****************************************************************************/
bool SPISetup_EnableSPI(SPI_Module_t WhichModule)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    pSPICON->ON = 1;   //enable SPI
    SPIOperate_HasSS1_Risen(); //clear this interrupt
  }
  return ReturnVal;
  
}
/****************************************************************************
 Function
    SPIOperate_SPI1_Send8

 Description
   Writes the 8-bit data to the selected SPI Module data register
  Does not check if there is room in the buffer.
  Note: separate functions provided for SPI1 & SPI2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
void SPIOperate_SPI1_Send8(uint8_t TheData)
{
  // not needed for ME218a Labs
  SPI1BUF = TheData;
  SPI1_hasWritten = true; // for SPIbusy event checker
}

/****************************************************************************
 Function
    SPIOperate_SPI1_Send16

 Description
   Writes the 16-bit data to the SPI1 Module data register
  Does not check if there is room in the buffer.
  Note: separate functions provided for SPI1 & SPI2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
void SPIOperate_SPI1_Send16( uint16_t TheData)
{
  SPI1BUF = TheData;       
  SPI1_hasWritten = true; // for SPIbusy event checker
}
/****************************************************************************
 Function
    SPIOperate_SPI1_Send32

 Description
   Writes the 32-bit data to the selected SPI Module data register
  Does not check if there is room in the buffer.
  Note: separate functions provided for SPI1 & SPI2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
void SPIOperate_SPI1_Send32(uint32_t TheData)
{
  // not needed for ME218a Labs
  SPI1BUF = TheData;       
  SPI1_hasWritten = true; // for SPIbusy event checker
}

/****************************************************************************
 Function
    SPIOperate_SPI1_Send8Wait

 Description
   Writes the 8-bit data to the selected SPI Module data register and waits
   for the SS line to rise. NOTE: this is blocking code and should only be 
   used when the bit-time on the SPI is sufficiently fast so as need to wait
   less than 200 micro-seconds to complete.  
   Does not check if there is room in the buffer.
  Note: separate functions provided for SPI1 & SPI2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
void SPIOperate_SPI1_Send8Wait(uint8_t TheData)
{
  // not needed for ME218a Labs
}

/****************************************************************************
  Function
    SPIOperate_SPI1_Send16Wait

  Description
    Writes the 16-bit data to the SPI1 Module data register and waits
    for the SS1 line to rise. NOTE: this is blocking code and should only be 
    used when the bit-time on the SPI is sufficiently fast so as need to wait
    less than 200 micro-seconds to complete.
    Does not check if there is room in the buffer.
  Note: separate functions provided for SPI1 & SPI2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
void SPIOperate_SPI1_Send16Wait( uint16_t TheData)
{
  // Your Code goes here :-)
  SPI1BUF = TheData;       
  while (SPI1STATbits.SPIBUSY) {} // wait to finish
  for (uint16_t i = 0; i < 10; i++) {} 
  
  while(SPIOperate_HasSS1_Risen()==false){} //wait until it rises
}
/****************************************************************************
 Function
    SPIOperate_SPI1_Send32Wait

 Description
   Writes the 32-bit data to the selected SPI Module data register and waits
   for the SS line to rise. NOTE: this is blocking code and should only be 
   used when the bit-time on the SPI is sufficiently fast so as need to wait
   less than 200 micro-seconds to complete.
  Does not check if there is room in the buffer.
  Note: separate functions provided for SPI1 & SPI2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
void SPIOperate_SPI1_Send32Wait(uint32_t TheData)
{
  // not needed for ME218a Labs
  
}
/****************************************************************************
 Function
    SPIOperate_ReadData

 Description
   Reads the data register for the selected SPI Module. Note: If the selected
   module is in 8-bit or 16-bit mode, then you should cast the result of this
   function to a uint8_t or uint16_t before assignment to a result variable.   
****************************************************************************/
uint32_t SPIOperate_ReadData(SPI_Module_t WhichModule)
{
  uint32_t ReturnVal = 0;
  // not needed for ME218a Labs
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    ReturnVal = *(pSPIBUF);
  }
  return ReturnVal;
}
/****************************************************************************
 Function
    SPIOperate_HasSS1_Risen
 Description
   Tests if the SS1 line has risen since the last time this
   function was called.
   Note: This is an event checking function,     not a state test. If the SS line 
   is found to have risen, then the hardware will be reset until the next time
   that data is written to the SPI module. After a call to this function 
   returns true, subsequent calls will return false until new data is written
   and another rising edge on the SS line is detected.
  Note: separate functions provided for SS1 & SS2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
bool SPIOperate_HasSS1_Risen(void)
{
  bool ReturnVal = false;

  if(1 == IFS0bits.INT4IF)
  {// we read a change! yay!
    ReturnVal = true;
    IFS0CLR = _IFS0_INT4IF_MASK;      // clear any pending flag
    // IFS0bits.INT4IF = 0; //clear bit for next transfer
  }
  ;
  return ReturnVal;
}

/****************************************************************************
 Function
    SPIOperate_HasSS2_Risen
 Description
   Tests if the SS2 line has gone low then back high since the last time this
   function was called.
   Note: This is an event checking function, not a state test. If the SS line 
   is found to have risen, then the hardware will be reset until the next time
   that data is written to the SPI module. After a call to this function 
   returns true, subsequent calls will return false until new data is written
   and another rising edge on the SS line is detected.
  Note: separate functions provided for SS1 & SS2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
bool SPIOperate_HasSS2_Risen(void)
{
  // not needed for ME218a
  bool ReturnVal = false;
  // Your Code goes here :-)
  if(1 == IFS0bits.INT1IF)
  {// we read a change! yay!
    ReturnVal = true;
    IFS0bits.INT1IF = 0; //clear bit for next transfer
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    SPIOperate_SPI1_FinishedTranmission, SPIOperate_SPI2_FinishedTranmission
 Description
   Tests if the SPI transfer has finished.  This is an event checking function 
   because the SPICON BUSY bit is auto-clearing, so we just check the state that
   should change as we need.  Janky, but i odon't make the rules.
   Note: This is an event checking function,     not a state test. If the SS line 
   is found to have risen, then the hardware will be reset until the next time
   that data is written to the SPI module. After a call to this function 
   returns true, subsequent calls will return false until new data is written
   and another rising edge on the SS line is detected.
  Note: separate functions provided for SS1 & SS2 in order to speed operation
  and allow the SPI to be run at higher bit rates
****************************************************************************/
bool SPIOperate_SPI1_FinishedTranmission(void)
{
  bool ReturnVal = false;
  // Your Code goes here :-)
  
  //check if currently expecting to be writing
  if (1 == SPI1_hasWritten) 
  {
    if(0 == SPI1STATbits.SPIBUSY)
    {// we read a change! yay!
      ReturnVal = true;
      SPI1_hasWritten = false; //no longer writing
    }
  }
  return ReturnVal;
}

bool SPIOperate_SPI2_FinishedTranmission(void)
{
  bool ReturnVal = false;
  // Your Code goes here :-)

  //check if currently expecting to be writing
  if (1 == SPI2_hasWritten) 
  {
    if(0 == SPI2STATbits.SPIBUSY)
    {// we read a change! yay!
      ReturnVal = true;
      SPI2_hasWritten = false; //no longer writing
    }
  }
  return ReturnVal;
}

bool SPIOperate_clearBuffer(SPI_Module_t WhichModule)
{
  bool ReturnVal = true;
  // Your Code goes here :-)
  if (false == isSPI_ModuleLegal(WhichModule))
  {
    ReturnVal = false; //exit
  }
  else
  {// legal module, continue
    selectModuleRegisters(WhichModule); //choose module SFRs

    //clear overflow bit
    pSPISTAT->SPIROV = 0;

    for (uint8_t i = 0; i < 32; i++)
    {
      //read the buffre a bunch to clear it out
      uint32_t val = SPIOperate_ReadData(WhichModule);
    }
  }
  return ReturnVal;
}


//*********************************
// private functions
//*********************************
/****************************************************************************
 Function
    selectModuleRegisters

 Description
   based in the requested module, initializes the pointers to the various
   SPI module registers.
****************************************************************************/
static void selectModuleRegisters(SPI_Module_t WhichModule)
{
  if (SPI_SPI1 == WhichModule)
  {
    pSPICON = (__SPI1CONbits_t *)&SPI1CON;
    pSPICON2 = (__SPI1CON2bits_t *)&SPI1CON2;
    pSPIBRG = &SPI1BRG;
    pSPIBUF = &SPI1BUF;
    pSPISTAT = (__SPI1STATbits_t *)&SPI1STAT;
  }else if (SPI_SPI2 == WhichModule)
  {
    pSPICON = (__SPI1CONbits_t *)&SPI2CON;
    pSPICON2 = (__SPI1CON2bits_t *)&SPI2CON2;
    pSPIBRG = &SPI2BRG;
    pSPIBUF = &SPI2BUF;
    pSPISTAT = (__SPI1STATbits_t *)&SPI2STAT;
  }
}

/****************************************************************************
 Function
    isSPI_ModuleLegal

 Description
   Compares the requested module to the legal modules.
   
****************************************************************************/
static bool isSPI_ModuleLegal( SPI_Module_t WhichModule)
{
  // Your Code goes here :-)
  bool returnVal = false;

  if((WhichModule == SPI_SPI1) || (WhichModule == SPI_SPI2) )
  {//module matches with something!
    returnVal = true;
  }
  return returnVal; //exit
}

/****************************************************************************
 Function
    isSS_OutputPinLegal

 Description
   Loops through the LegalSSOutPins array comparing the requested pin to each
   of the entries in the array. If a match is found, sets RetrnVal to true and 
   breaks out of the loop.
****************************************************************************/
static bool isSS_OutputPinLegal(SPI_Module_t WhichModule, SPI_PinMap_t WhichPin)
{
  bool ReturnVal = false;
  uint8_t index;
  
  for( index = 0; 
       index <= ((sizeof(LegalSSOutPins[WhichModule])/sizeof(LegalSSOutPins[WhichModule][0]))-1);
       index++)
  {
    if (LegalSSOutPins[WhichModule][index] == WhichPin)
    {
      ReturnVal = true;
      break;
    }
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    isSDOPinLegal

 Description
   Loops through the LegalSDOxPins array comparing the requested pin to each
   of the entries in the array. If a match is found, sets RetrnVal to true and 
   breaks out of the loop.
****************************************************************************/
static bool isSDOPinLegal( SPI_PinMap_t WhichPin)
{
  bool ReturnVal = false;
  // Your Code goes here :-)

  for (uint8_t ind = 0; 
      ind < ((sizeof(LegalSDOxPins)/sizeof(LegalSDOxPins[0]))); 
      ind++)
  {
    if (LegalSDOxPins[ind] == WhichPin)
    {
      ReturnVal = true;
      break; //exit the loop now, we know it's ok
    }
  }
  
  return ReturnVal;
}

/****************************************************************************
 Function
    isSDIPinLegal

 Description
   Loops through the LegalSDIxPins array comparing the requested pin to each
   of the entries in the array. If a match is found, sets RetrnVal to true and 
   breaks out of the loop.
****************************************************************************/
static bool isSDIPinLegal(SPI_Module_t WhichModule, SPI_PinMap_t WhichPin)
{
  bool ReturnVal = false;
  // Your Code goes here :-)

  for (uint8_t ind = 0; 
      ind < ((sizeof(LegalSDIxPins[WhichModule])/sizeof(LegalSDIxPins[WhichModule][0]))); 
      ind++)
  {
    if (LegalSDIxPins[WhichModule][ind] == WhichPin)
    {
      ReturnVal = true;
      break; //exit the loop now, we know it's ok
    }
  }
  
  return ReturnVal;
}



void SPI_testharness_setup(void)
{
  /**
    *  Configure SPI
    * Things to configure:
    *  - 16 bit transfers
    *  - CKE = 0
    *  - CKP = 1
    *  - Enhanced Buffer Mode
    *  - Leader (master) Mode
    *  - master SS control
    *  - Active Low polarity on SS
    *  - PBCLock
    *  - 10kHz data rate 
    
    * ****
    * Split into each specific function call
    */


  // SPI1CON = 0;                //clear all SPICON data

  #define NO_HAL    0 //don't use hall, use register bang

  //BasicConfig
  #if (NO_HAL) //test w/o HAL
    SPI1CONbits.ON = 0; //turn off
    SPI1CONbits.MCLKSEL = 0; //use PBclock
    SPI1CONbits.FRMEN = 0; //framed mode off
    SPI1CON2bits.AUDEN = 0; //disable audio
  // SPI1BUF; //clear buffer
  #elif(1)
    SPISetup_BasicConfig(SPI_SPI1);
  #endif

  

  //SetLeader
  #if (NO_HAL)
    SPI1CONbits.MSTEN = 1; //leader (master) mode enable
    SPI1CONbits.SMP = 0; //sample in middle
  #elif(1)
    SPISetup_SetLeader(SPI_SPI1, SPI_SMP_MID);
  #endif

  //SetBitTime
  #if (NO_HAL)
    //calculate 10kHz rate
    uint32_t BRGsetting = (20e6)/(2*10e3)-1; // BRG setting (should be 999?)
    SPI1BRG = BRGsetting; //write into clock divider
  #elif(1)
    SPISetup_SetBitTime(SPI_SPI1, 100e3);
  #endif

  //MapSSOutput
  #if (NO_HAL)
    SPI1CONbits.MSSEN = 1; //leader mode SS control
    ANSELACLR = BIT0HI;
    TRISACLR = BIT0HI ;
    RPA0R = (uint32_t) 0b0011; //SS1 = RA0
  #elif(1)
    SPISetup_MapSSOutput(SPI_SPI1, SPI_RPA0);
  #endif

  //MapSDInput
  #if (NO_HAL) 
    SPI1CONbits.DISSDI = 1; //enable sdi pin?
  #elif (1)
    SPISetup_MapSDInput(SPI_SPI1, SPI_NO_PIN);
  #endif

  //MapSDOutput
  #if (NO_HAL)
    SPI1CONbits.DISSDO = 0; //enable SDO pin
    ANSELACLR = BIT1HI;
    TRISACLR = BIT1HI;
    RPA1R = (uint32_t) 0b0011; //SDO1 = RA1
  #elif(1)
    SPISetup_MapSDOutput(SPI_SPI1, SPI_RPA1);
  #endif

  //SetClockIdleState
  #if (NO_HAL)
    SPI1CONbits.CKP = 1; //clock polarity, idle low
  #elif(1)
    SPISetup_SetClockIdleState(SPI_SPI1, SPI_CLK_HI);
  #endif

  //SetActiveEdge
  #if (NO_HAL)
    SPI1CONbits.CKE = 0; //clock edge select, idle to active
  #elif(1)
    SPISetup_SetActiveEdge(SPI_SPI1, SPI_SECOND_EDGE);
  #endif

  //SetXferWidth
  #if (NO_HAL)
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 1; //16 bit transfers
  #elif(1)
    SPISetup_SetXferWidth(SPI_SPI1, SPI_16BIT);
  #endif

  //SetEnhancedBuffer
  #if (NO_HAL)
    SPI1CONbits.ENHBUF = 1; //enable enhanced buffer
  #elif(1)
    SPISetEnhancedBuffer(SPI_SPI1, true);
  #endif

  //clock as output?
  #if (NO_HAL)
    ANSELBCLR = BIT14HI;
    TRISBCLR = BIT14HI;
  #endif


  //other things??
  // SPI1CONbits.FRMPOL = 0; //active low pulse?

  // SPI1STATbits.SPIROV = 0; //clear the overflow bit

  //SPI ON
  #if (NO_HAL)
    SPI1CONbits.ON = 1; //enable SPI!! whoo!
  #elif(1)
    SPISetup_EnableSPI(SPI_SPI1);
  #endif

}

void SPI_testharness_loop(void)
{
  uint16_t toSend[]= {0x004D, 0x0045, 0x0032, 0x0031, 0x0038, 0x804D,
     0x8045, 0x8032, 0x8031, 0x8038};
  uint8_t toSendSize = sizeof(toSend)/sizeof(toSend[0]); //how many bytes?

  for (uint8_t i = 0; i < toSendSize; i++)
  {
    #if (NO_HAL)
      SPI1BUF = toSend[i];
    #elif (1)
      if(i < toSendSize-1)
      {//one before done
        SPIOperate_SPI1_Send16(toSend[i]);
      }
      else
      { //send last one and wait
        SPIOperate_SPI1_Send16Wait(toSend[i]);
      }
    #elif (1)
      
    #endif
  }
  for (uint16_t i = 0; i < 400; i++) {} 


  #if(NO_HAL) //add waiters?
    while (SPI1STATbits.SPIBUSY) {} // wait to finish
    for (uint16_t i = 0; i < 400; i++) {} 
  #endif

}
