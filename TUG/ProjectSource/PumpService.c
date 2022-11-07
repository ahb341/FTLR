/****************************************************************************
 Module
   PumpService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <xc.h>
#include <sys/attribs.h>
#include <proc/p32mx170f256b.h>

#include "ES_Configure.h"
#include "ES_Framework.h"

#include "PIC32PortHAL.h"

#include "PumpService.h"

/*----------------------------- Module Defines ----------------------------*/
#define PWM_PERIOD 999              // PWM period (in ticks)

#define MAX_DUTY PWM_PERIOD
#define MIN_DUTY (0.3*PWM_PERIOD)
#define LIN_CONV(x) (x*(MAX_DUTY-MIN_DUTY)/PWM_PERIOD+MIN_DUTY)

#define MAX_ACTIVE_PUMPS    3 // for current draw limitation
#define ACTIVE_PUMPS_FUEL   2 // for fuel consumption tracking

#define ONE_SEC 1000
#define FUEL_CAPACITY 10*ONE_SEC
#define FUEL_USAGE_INTERVAL ONE_SEC/100 // How often usage is calculated (in ms)

#define FULL_TANK (int)PWM_PERIOD*(int)ACTIVE_PUMPS_FUEL*(int)FUEL_CAPACITY

#define TRANSL_THRESH 60

#define INPUT_DEADBAND  10 // below this number, don't set pump values

// Pump 1 (FRONT LEFT) - Output Compare 2
#define PUMP_1_PORT     _Port_B
#define PUMP_1_PIN      _Pin_5
#define PUMP_1_RPNR     RPB5R
#define PUMP_1_OCXRS    OC2RS

// Pump 2 (FRONT RIGHT) - Output Compare 1
#define PUMP_2_PORT     _Port_B
#define PUMP_2_PIN      _Pin_4
#define PUMP_2_RPNR     RPB4R
#define PUMP_2_OCXRS    OC1RS

// Pump 3 (BACK RIGHT) - Output Compare 3
#define PUMP_3_PORT     _Port_B
#define PUMP_3_PIN      _Pin_9
#define PUMP_3_RPNR     RPB9R
#define PUMP_3_OCXRS    OC3RS

// Pump 4 (BACK LEFT) - Output Compare 4
#define PUMP_4_PORT     _Port_A
#define PUMP_4_PIN      _Pin_4
#define PUMP_4_RPNR     RPA4R
#define PUMP_4_OCXRS    OC4RS

#define STARTING_DC 50

// print/debug statements
//#define WASD
#define DEBUG_NormalizePrints
#define DEBUG_PrintThrustVals
// #define Announce_FuelLevel
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void _PumpDrive_Init(void);
static void NewDirection(char ThisChar);
static void UpdateDutyCycles(void);
static void NormalizeThrusts(void);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static PumpState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

static uint16_t CurrentDutyCycle;

enum Pumps      // Numbered clockwise starting with front-left
{
    FL_Pump = 1,
    FR_Pump,
    BR_Pump,
    BL_Pump
};

static uint32_t PumpThrusts[6];

static bool FL_PumpState;
static bool FR_PumpState;
static bool BR_PumpState;
static bool BL_PumpState;

static int8_t X_ThrustVal;
static int8_t Y_ThrustVal;
static int8_t Z_ThrustVal;

static bool FuelLeft;
static uint16_t FuelUsageRate;
static uint32_t FuelConsumed;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitPumpService

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
bool InitPumpService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = PumpInitPState;
  
  CurrentDutyCycle = STARTING_DC;
  
  FL_PumpState = 0;
  FR_PumpState = 0;
  BR_PumpState = 0;
  BL_PumpState = 0;
  
  X_ThrustVal = 0;
  Y_ThrustVal = 0;
  Z_ThrustVal = 0;
  
  FuelLeft = true;
  FuelUsageRate = 0;
  FuelConsumed = 0;
  
  _PumpDrive_Init();
  
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
     PostPumpService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostPumpService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunPumpService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunPumpService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch (CurrentState)
  {
    case PumpInitPState:        // If current state is initial Psedudo State
    {
        if (ES_INIT == ThisEvent.EventType)
        {
            ES_Timer_InitTimer(FUEL_USAGE_TIMER,FUEL_USAGE_INTERVAL);
            
            // now put the machine into the actual initial state
            CurrentState = FuelRemaining;
        }
    }
    break;
    
    case FuelRemaining:
    {
        switch (ThisEvent.EventType)
        {
            case ES_INIT:
            {
                ES_Timer_InitTimer(FUEL_USAGE_TIMER,FUEL_USAGE_INTERVAL);
            }
            break;

            case EV_THRUST_UPDATE:
            {
                // printf("EV_THRUST_UPDATE received.\n\n\r");
                NormalizeThrusts();
                UpdateDutyCycles();
            }
            break;

            case ES_RefuelCmd:
            {
                if (0 != ThisEvent.EventParam)
                {
                    printf("***TANK REFUELED*** (from rePair) \n\n\r");
                    FuelLeft = true;
                    FuelConsumed = 0;
                    
                    CurrentState = FuelRemaining;
                    
                    ES_Timer_InitTimer(FUEL_USAGE_TIMER,FUEL_USAGE_INTERVAL);
                }
                break;
            }

            case ES_TIMEOUT:
            {
                if (FUEL_USAGE_TIMER == ThisEvent.EventParam)
                {
                    FuelConsumed += FUEL_USAGE_INTERVAL*FuelUsageRate;
//                    printf("Fuel consumed: %u\n\r",FuelConsumed);
//                    printf("Total fuel: %u\n\n\r",FULL_TANK);

                    if (FuelConsumed >= FULL_TANK)
                    {
                        printf("***OUT OF FUEL***\n\n\r");
                        FuelLeft = false;

                        UpdateThrustVectors(0,0,0);

                        for (int i = FL_Pump; i <= BL_Pump; i++)
                        {
                            PumpThrusts[i] = 0;
                        }

                        UpdateDutyCycles();

                        FuelUsageRate = 0;
                        
                        CurrentState = WaitingForRefuel;
                    }

                        #ifdef Announce_FuelLevel
                            printf("Fuel left:%d\n\r", getRemainingFuel());
                        #endif /* Announce_FuelLevel */
                    
                    ES_Timer_InitTimer(FUEL_USAGE_TIMER,FUEL_USAGE_INTERVAL);
                }
            }
            break;

            case ES_NEW_KEY:
            {
                switch (ThisEvent.EventParam)
                {
                    case 'z':
                    {
                        UpdateThrustVectors(0,0,0);

                        ES_Event_t NewEvent;
                        NewEvent.EventType = EV_THRUST_UPDATE;
                        ES_PostToService(MyPriority, NewEvent);
                    }
                    break;

                    case 'c':
                    {
                        UpdateThrustVectors(127,63,45);

                        ES_Event_t NewEvent;
                        NewEvent.EventType = EV_THRUST_UPDATE;
                        ES_PostToService(MyPriority, NewEvent);
                    }
                    break;

                    case 'v':
                    {
                        UpdateThrustVectors(85,-43,-95);

                        ES_Event_t NewEvent;
                        NewEvent.EventType = EV_THRUST_UPDATE;
                        ES_PostToService(MyPriority, NewEvent);
                    }
                    break;

                    case 'n':
                    {
                        UpdateThrustVectors(127,0,0);

                        ES_Event_t NewEvent;
                        NewEvent.EventType = EV_THRUST_UPDATE;
                        ES_PostToService(MyPriority, NewEvent);
                    }
                    break;

                    case 'm':
                    {
                        UpdateThrustVectors(127,127,127);

                        ES_Event_t NewEvent;
                        NewEvent.EventType = EV_THRUST_UPDATE;
                        ES_PostToService(MyPriority, NewEvent);
                    }
                    break;
                

                    #ifdef WASD
                    case ' ':
                    {
                        printf("\nPumps OFF\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case 'w':
                    {
                        printf("\nMoving forward...\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case 's':
                    {
                        printf("\nMoving backward...\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case 'a':
                    {
                        printf("\nMoving left...\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case 'd':
                    {
                        printf("\nMoving right...\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case 'k':
                    {
                        printf("\nRotating CCW...\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case 'l':
                    {
                        printf("\nRotating CW...\n\r");
                        NewDirection((char)ThisEvent.EventParam);
                        UpdateDutyCycles();
                    }
                    break;

                    case '`':
                    {
                        printf("\nDuty Cycle: 0.00\n\r");
                        CurrentDutyCycle = 0;
                        UpdateDutyCycles();
                    }
                    break;

                    case '1':
                    {
                        printf("\nDuty Cycle: 0.10\n\r");
                        CurrentDutyCycle = (int)(0.1*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '2':
                    {
                        printf("\nDuty Cycle: 0.20\n\r");
                        CurrentDutyCycle = (int)(0.2*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '3':
                    {
                        printf("\nDuty Cycle: 0.30\n\r");
                        CurrentDutyCycle = (int)(0.3*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '4':
                    {
                        printf("\nDuty Cycle: 0.40\n\r");
                        CurrentDutyCycle = (int)(0.4*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '5':
                    {
                        printf("\nDuty Cycle: 0.50\n\r");
                        CurrentDutyCycle = (int)(0.5*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '6':
                    {
                        printf("\nDuty Cycle: 0.60\n\r");
                        CurrentDutyCycle = (int)(0.6*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '7':
                    {
                        printf("\nDuty Cycle: 0.70\n\r");
                        CurrentDutyCycle = (int)(0.7*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '8':
                    {
                        printf("\nDuty Cycle: 0.80\n\r");
                        CurrentDutyCycle = (int)(0.8*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '9':
                    {
                        printf("\nDuty Cycle: 0.90\n\r");
                        CurrentDutyCycle = (int)(0.9*PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;

                    case '0':
                    {
                        printf("\nDuty Cycle: 1.00\n\r");
                        CurrentDutyCycle = (int)(PWM_PERIOD);
                        UpdateDutyCycles();
                    }
                    break;
                    #endif

                    default:
                        break;
                }
            }
            break;

            default:
                break;
        }
    }
    break;
    
    case WaitingForRefuel:
    {
        switch (ThisEvent.EventType)
        {
            case ES_NEW_KEY:
            {
                switch (ThisEvent.EventParam)
                {
                    case 'r':
                    {
                        printf("***TANK REFUELED***\n\n\r");
                        FuelLeft = true;
                        FuelConsumed = 0;

                        CurrentState = FuelRemaining;

                        ES_Timer_InitTimer(FUEL_USAGE_TIMER,FUEL_USAGE_INTERVAL);
                    }
                    break;
                    
                    default:
                        break;
                }
            }
            break;

            case ES_RefuelCmd:
            {
                printf("***TANK REFUELED*** (from XBee) \n\n\r");
                FuelLeft = true;
                FuelConsumed = 0;
                
                CurrentState = FuelRemaining;
                
                ES_Timer_InitTimer(FUEL_USAGE_TIMER,FUEL_USAGE_INTERVAL);
                break;
            }
            
            default:
                break;
        }
    }
    break;
    
    default:
          break;
  }
  
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryPumpService

 Parameters
     None

 Returns
     PumpState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
PumpState_t QueryPumpService(void)
{
  return CurrentState;
}

// *********************************************************
// Thrust vectors setter function
// *********************************************************
void UpdateThrustVectors(int8_t NewCtrlX, int8_t NewCtrlY, int8_t NewCtrlZ)
{
    X_ThrustVal = NewCtrlX;
    Y_ThrustVal = NewCtrlY;
    Z_ThrustVal = NewCtrlZ;
    
    #ifdef DEBUG_PrintThrustVals
        printf("ThrustVals updated: %i, %i, %i\n\n\r",X_ThrustVal,Y_ThrustVal,Z_ThrustVal);
    #endif /* DEBUG_PrintThrustVals */
}

/** getRemainingFuel()
 * 
 * How much fuel is left, getter function for communications
 */
uint8_t getRemainingFuel()
{
    uint8_t returnVal = 0; // assume it has no fuel
    if (FuelLeft)
    {
        float percentRemaining = (FULL_TANK - FuelConsumed) / ((float) FULL_TANK);
        // printf("\t percent: %f\n\r", percentRemaining);
        // printf("\t consumed: %d / %d\n\r", FuelConsumed, (int) FULL_TANK);
        returnVal = percentRemaining * 255.0;
    }
    return returnVal;
}


/***************************************************************************
 private functions
 ***************************************************************************/

// *********************************************************
// Pump PWM-Drive initializer
// *********************************************************
static void _PumpDrive_Init(void)
{
    // Timer 2 configuration
    T2CONbits.ON = 0;               // Disable TMR2
    T2CONbits.TCS = 0;				// Select PBCLK as clock source
	T2CONbits.TGATE = 0;			// Disable gated time accumulation
	T2CONbits.TCKPS = 0b010;        // Set prescale to 1:4
    PR2 = PWM_PERIOD;               // Set PWM period in ticks
    TMR2 = 0;                       // Zero TMR2
    
    // Configure pump control pins as digital outputs
    PortSetup_ConfigureDigitalOutputs(PUMP_1_PORT,PUMP_1_PIN);
    PortSetup_ConfigureDigitalOutputs(PUMP_2_PORT,PUMP_2_PIN);
    PortSetup_ConfigureDigitalOutputs(PUMP_3_PORT,PUMP_3_PIN);
    PortSetup_ConfigureDigitalOutputs(PUMP_4_PORT,PUMP_4_PIN);
    
    // Assign pump control pins to OC modules
    PUMP_1_RPNR = 0b0101;
    PUMP_2_RPNR = 0b0101;
    PUMP_3_RPNR = 0b0101;
    PUMP_4_RPNR = 0b0101;
    
    // Pump 1: Output Capture 1 PWM configuration
    OC1CONbits.ON = 0;              // Disable OC1
    OC1CONbits.OC32 = 0;            // 16bit compare mode
    OC1CONbits.OCTSEL = 0;          // Assign to TMR2
    OC1CONbits.OCM = 0b110;         // PWM mode, fault disabled
    OC1R = 0;                       // Set initial duty cycle
    OC1RS = 0;                      // Set on-deck duty cycle
    OC1CONbits.ON = 1;              // Enable OC1
    
    // Pump 2: Output Capture 3 PWM configuration
    OC3CONbits.ON = 0;              // Disable OC3
    OC3CONbits.OC32 = 0;            // 16bit compare mode
    OC3CONbits.OCTSEL = 0;          // Assign to TMR2
    OC3CONbits.OCM = 0b110;         // PWM mode, fault disabled
    OC3R = 0;                       // Set initial duty cycle
    OC3RS = 0;                      // Set on-deck duty cycle
    OC3CONbits.ON = 1;              // Enable OC3
    
    // Pump 3: Output Capture 4 PWM configuration
    OC4CONbits.ON = 0;              // Disable OC4
    OC4CONbits.OC32 = 0;            // 16bit compare mode
    OC4CONbits.OCTSEL = 0;          // Assign to TMR2
    OC4CONbits.OCM = 0b110;         // PWM mode, fault disabled
    OC4R = 0;                       // Set initial duty cycle
    OC4RS = 0;                      // Set on-deck duty cycle
    OC4CONbits.ON = 1;              // Enable OC4
    
    // Pump 4: Output Capture 2 PWM configuration
    OC2CONbits.ON = 0;              // Disable OC2
    OC2CONbits.OC32 = 0;            // 16bit compare mode
    OC2CONbits.OCTSEL = 0;          // Assign to TMR2
    OC2CONbits.OCM = 0b110;         // PWM mode, fault disabled
    OC2R = 0;                       // Set initial duty cycle
    OC2RS = 0;                      // Set on-deck duty cycle
    OC2CONbits.ON = 1;              // Enable OC2
    
    T2CONbits.ON = 1;               // Enable TMR2
}

#ifdef WASD
// *********************************************************
// Helper function for changing direction of movement
// *********************************************************
static void NewDirection(char ThisChar)
{
    switch (ThisChar)
    {
        case ' ':
        {
            FL_PumpState = 0;
            FR_PumpState = 0;
            BR_PumpState = 0;
            BL_PumpState = 0;
        }
        break;
        
        case 'w':
        {
            FL_PumpState = 0;
            FR_PumpState = 0;
            BR_PumpState = 1;
            BL_PumpState = 1;
        }
        break;
        
        case 's':
        {
            FL_PumpState = 1;
            FR_PumpState = 1;
            BR_PumpState = 0;
            BL_PumpState = 0;
        }
        break;
        
        case 'a':
        {
            FL_PumpState = 1;
            FR_PumpState = 0;
            BR_PumpState = 0;
            BL_PumpState = 1;
        }
        break;
        
        case 'd':
        {
            FL_PumpState = 0;
            FR_PumpState = 1;
            BR_PumpState = 1;
            BL_PumpState = 0;
        }
        break;
        
        case 'k':
        {
            FL_PumpState = 1;
            FR_PumpState = 0;
            BR_PumpState = 1;
            BL_PumpState = 0;
        }
        break;
        
        case 'l':
        {
            FL_PumpState = 0;
            FR_PumpState = 1;
            BR_PumpState = 0;
            BL_PumpState = 1;
        }
        break;
        
        default:
            break;
    }
}
#endif

// *********************************************************
// Helper function for updating duty cycles of active pumps
// *********************************************************
static void UpdateDutyCycles(void)
{
#ifdef WASD
    PUMP_1_OCXRS = FL_PumpState*CurrentDutyCycle;
    PUMP_2_OCXRS = FR_PumpState*CurrentDutyCycle;
    PUMP_3_OCXRS = BR_PumpState*CurrentDutyCycle;
    PUMP_4_OCXRS = BL_PumpState*CurrentDutyCycle;
#else
    PUMP_1_OCXRS = PumpThrusts[FL_Pump];
    PUMP_2_OCXRS = PumpThrusts[FR_Pump];
    PUMP_3_OCXRS = PumpThrusts[BR_Pump];
    PUMP_4_OCXRS = PumpThrusts[BL_Pump];
#endif
}

// *********************************************************
// Helper function for normalizing pump thrust/DC values
// *********************************************************
static void NormalizeThrusts(void)
{
    for (int i = FL_Pump; i <= BL_Pump; i++)    // Clear array of pump thrusts
    {
        PumpThrusts[i] = 0;
    }
    
    FuelUsageRate = 0;                  // Clear total rate of fuel consumption
    
    float RotationWeight = 1;           // For reducing rotation magnitude while moving XY
    
    if (abs(X_ThrustVal) > INPUT_DEADBAND)
    {
        if (0 < X_ThrustVal)        // If forward-translation movement requested
        {
            PumpThrusts[BR_Pump] += X_ThrustVal;
            PumpThrusts[BL_Pump] += X_ThrustVal;
        }
        else if (0 > X_ThrustVal)  // If backward-translation movement requested
        {
            PumpThrusts[FL_Pump] += abs(X_ThrustVal);
            PumpThrusts[FR_Pump] += abs(X_ThrustVal);
        }
        
        if (abs(X_ThrustVal) >= TRANSL_THRESH)
        {
            RotationWeight -= 0.25;
        }
    }
    
    if (abs(Y_ThrustVal) > INPUT_DEADBAND)
    {
        if (0 < Y_ThrustVal)        // If right-translation movement requested
        {
            PumpThrusts[FR_Pump] += Y_ThrustVal;
            PumpThrusts[BR_Pump] += Y_ThrustVal;
        }else if (0 > Y_ThrustVal)  // If left-translation movement requested
        {
            PumpThrusts[FL_Pump] += abs(Y_ThrustVal);
            PumpThrusts[BL_Pump] += abs(Y_ThrustVal);
        }
        
        if (abs(Y_ThrustVal) >= TRANSL_THRESH)
        {
            RotationWeight -= 0.25;
        }
    }
    
    if (abs(Z_ThrustVal) > INPUT_DEADBAND)
    {
        if (0 < Z_ThrustVal)        // If clockwise-rotation movement requested
        {
            PumpThrusts[FR_Pump] += Z_ThrustVal*RotationWeight;
            PumpThrusts[BL_Pump] += Z_ThrustVal*RotationWeight;
        }else if (0 > Z_ThrustVal)  // If counterclockwise-rotation movement requested
        {
            PumpThrusts[FL_Pump] += abs(Z_ThrustVal)*RotationWeight;
            PumpThrusts[BR_Pump] += abs(Z_ThrustVal)*RotationWeight;
        }
    }
    
    #ifdef DEBUG_NormalizePrints
        printf("Thrust Summations:\n\r");
        printf("Front-Left  Pump Requested Thrust = %i\n\r",PumpThrusts[FL_Pump]);
        printf("Front-Right Pump Requested Thrust = %i\n\r",PumpThrusts[FR_Pump]);
        printf("Back-Right  Pump Requested Thrust = %i\n\r",PumpThrusts[BR_Pump]);
        printf("Back-Left   Pump Requested Thrust = %i\n\r",PumpThrusts[BL_Pump]);
        printf("\n");
    #endif /* DEBUG_NormalizePrints */
    
    uint32_t ThrustSum = 0;
    uint8_t PumpsOn = 0;
    for (int i = FL_Pump; i <= BL_Pump; i++)
    {
        if (0 != PumpThrusts[i])    // If current pump is requested
        {
            ThrustSum += PumpThrusts[i];                    // Add to total requested thrust
            PumpsOn++;                                      // Increment number of pumps needed
        }
    }
    #ifdef DEBUG_NormalizePrints
        printf("Total Requested Thrust: %i\n\r",ThrustSum);
        printf("Number of Pumps Needed: %i\n\n\r",PumpsOn);
    #endif /* DEBUG_NormalizePrints */
    
    for (int i = FL_Pump; i <= BL_Pump; i++)
    {
        if (0 != PumpThrusts[i])    // If current pump is requested
        {
            PumpThrusts[i] *= MAX_ACTIVE_PUMPS*PWM_PERIOD;  // 200% DC available
            PumpThrusts[i] /= 127*(PumpsOn-1);              // Normalize by number of movements requested
            PumpThrusts[i] /= PumpsOn;                      // Split across requested pumps
            
            FuelUsageRate += PumpThrusts[i];                // Add to total rate of fuel consumption
            
            PumpThrusts[i] = LIN_CONV(PumpThrusts[i]);      // Linear conversion over range of min:max DC
            //printf("Converted value: %i\n\n\r",PumpThrusts[i]);
        }
    }
    //printf("FuelUsageRate = %i\n\n\r",FuelUsageRate);
    #ifdef DEBUG_NormalizePrints
        printf("Normalized Values:\n\r");
        printf("Front-Left  Pump Normalized DC = %u\n\r",PumpThrusts[FL_Pump]);
        printf("Front-Right Pump Normalized DC = %u\n\r",PumpThrusts[FR_Pump]);
        printf("Back-Right  Pump Normalized DC = %u\n\r",PumpThrusts[BR_Pump]);
        printf("Back-Left   Pump Normalized DC = %u\n\r",PumpThrusts[BL_Pump]);
        printf("\n");
    #endif /* DEBUG_NormalizePrints */
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

