/****************************************************************************
 Module
   TemplateService.c

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
// Events and Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"

// C Standard
#include <stdlib.h>

// HALs
#include "../../HALs/PIC32PortHAL.h"
#include "../../HALs/PIC32_AD_Lib.h"

// Project Libraries
#include "PilotSPI.h"
#include "Pilot_XBee.h"
#include "Pilot_CommComm.h"
#include "accelerometer.h"

// This Module
#include "PilotService.h"


/*----------------------------- Module Defines ----------------------------*/
// Accelerometer
#define ANGLE_THRESH 10 // [degrees]
#define THETA_THRESH 20 // [degrees/2] of horizontal change
#define REQ_MOVES 3

// Joystick
// Fwd: x=1023, Rev: x=0, Right: y=1023, Left: y=0
#define NUMPINS 4
#define MAX 1023
#define MID 510
#define MIN 0
#define TOL 25
#define CONTROL_MAX 127.0

#define TANK_MAX_THRESHOLD  0.95 // within this value, consider it "max"
#define TANK_MIN_THRESHOLD  0.0 // within this value, consider it "off"


#define JOYSTICK_TIMEOUT 200     // 200 ms

// Joystick 1
#define J1XPORT _Port_B
#define J1XPIN _Pin_13
#define J1XREAD PORTBbits.RB13
#define J1XANPIN BIT11HI
#define J1YPORT _Port_B
#define J1YPIN _Pin_15
#define J1YREAD PORTBbits.RB15
#define J1YANPIN BIT9HI
#define J1SWPORT _Port_B
#define J1SWPIN _Pin_4
#define J1SWREAD PORTBbits.RB4

// Joystick 2
#define J2XPORT _Port_B
#define J2XPIN _Pin_2
#define J2XREAD PORTBbits.RB2
#define J2XANPIN BIT4HI
#define J2YPORT _Port_B
#define J2YPIN _Pin_12
#define J2YREAD PORTBbits.RB12
#define J2YANPIN BIT12HI
#define J2SWPORT _Port_A
#define J2SWPIN _Pin_4
#define J2SWREAD PORTAbits.RA4

#define NUM_DRIVE_MODES 3
#define NUM_INPUT_MODES 3

// change accelerometer read style
#define ACCEL_SPHERICAL // use spherical coordinates to read total angle

//joystick tank mixing style
#define TANK_DRIVE_ARCADE_EXCEPTIONS

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static bool InitJoystick(void);
static void JoystickUpdate(void);
void calculateDanceMoves(uint16_t newAngles);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static PilotState_t CurrentState;
static uint8_t MyPriority;
static uint8_t NumMoves;
static int16_t CurrentAngle;
static uint32_t LastJoystickVal[NUMPINS];

static enum 
{
    Joy2_Y_Posn = 0,
    Joy1_Y_Posn,
    Joy1_X_Posn,
    Joy2_X_Posn,
} JoystickAxisPositions;

typedef enum
{
    Eco = 0,
    Sport,
    Turbo
} DriveMode_t;

typedef enum
{
    Arcade_2_stick, Arcade_1_stick, Tank_2_stick,
} InputMode_t;

const unsigned char DriveModes[NUM_DRIVE_MODES] = {'E','S','T'};
const uint8_t DrivePowerPercents[NUM_DRIVE_MODES] = {50, 75, 100};
static DriveMode_t DriveMode = Turbo; 
const char InputLabels[NUM_INPUT_MODES][2] = {{'A', '2'},
                                              {'A', '1'},
                                              {'T', 'K'}};
static InputMode_t InputMode = Arcade_2_stick; // default
const unsigned char Teams[7] = {'0','1','2','3','4','5','6'};
const unsigned char Unpaired = 'U';

static uint8_t Fuel = 255;

static uint16_t lastJoystickRead = 0;

static AccDataToSend_t sphericalAngles; // storing spherical angles
static uint8_t lastTheta = 0xff; //previous thetaValue, inits at max value


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

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
bool InitPilotService(uint8_t Priority)
{
    ES_Event_t ThisEvent;

    MyPriority = Priority;
    CurrentState = PilotInitState;
    NumMoves = 0;
    if (!InitJoystick()) return false;
    
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
     PostTemplateService

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
bool PostPilotService(ES_Event_t ThisEvent)
{
    return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

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
ES_Event_t RunPilotService(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent;
    ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

    switch (CurrentState)
    {
        case (PilotInitState):
        {
            if (ThisEvent.EventType == ES_INIT)
            {
                CurrentState = PilotWaiting;
                ES_Timer_InitTimer(JOYSTICK_TIMER,JOYSTICK_TIMEOUT);
            }
        }
        break;
        
        case (PilotWaiting):
        {
            // printf("\rPILOT WAITING\r\n");
            switch (ThisEvent.EventType)
            {

                case (ES_TIMEOUT):
                {
                  if (ThisEvent.EventParam == JOYSTICK_TIMER)
                  {
                    ADC_MultiRead(LastJoystickVal);
                    JoystickUpdate();               
                    ES_Timer_InitTimer(JOYSTICK_TIMER, JOYSTICK_TIMEOUT);
                  }
                }
                break;
                
                case (PS_DRIVE_MODE):
                {
                    if (DriveMode >= Turbo)
                    {
                        DriveMode = Eco;
                    }
                    else
                    {
                        DriveMode++;
                    }
                    PostPilotSPI((ES_Event_t) {PSPI_DRIVE_MODE,DriveModes[DriveMode]});
                }
                break;
                
                case (PS_INPUT_MODE):
                {
                    InputMode++; // increment
                    InputMode %= NUM_INPUT_MODES; // wraparound
                    ES_Event_t toPost = {PSPI_INPUT_MODE, 0};
                    toPost.EventParam = InputLabels[InputMode][0] << 8 |
                                        InputLabels[InputMode][1];
                    PostPilotSPI(toPost);
                }  
                break;
                
                case (PS_PAIRED):
                {
                    PostPilotSPI((ES_Event_t) {PSPI_PAIR_UNPAIR,
                                 Teams[getTUGlatchedIndex()]});
                }
                break;
                
                case (PS_UNPAIRED):
                {
                    PostPilotSPI((ES_Event_t) {PSPI_PAIR_UNPAIR,Unpaired});
                }
                break;
                
                case (PS_TEAM_SELECT):
                {
                    PostPilotSPI((ES_Event_t) {PSPI_TEAM_SELECT,
                                 Teams[getTUGpairingIndex()]});
                }
                break;
                
                case (PS_FUEL_UPDATE):
                {
                    Fuel = getFuelLevel(); // get from XBee
                    PostPilotSPI((ES_Event_t) {PSPI_FUEL_UPDATE,Fuel});
                }
                break;
                
                case (ES_NEW_KEY):
                {
                    if (ThisEvent.EventParam == 'f')
                    {
                        PostPilotService((ES_Event_t) {PS_REFUEL,0});
                    }
                    else if (ThisEvent.EventParam == 'e')
                    {
                        PostPilotService((ES_Event_t) {PS_DRIVE_MODE,0});
                    }
                    else if (ThisEvent.EventParam == 'p')
                    {
                        PostPilotService((ES_Event_t) {PS_PAIRED,0});
                    }
                    else if (ThisEvent.EventParam == 'u')
                    {
                        PostPilotService((ES_Event_t) {PS_UNPAIRED,0});
                    }
                    else if (ThisEvent.EventParam == 't')
                    {
                        PostPilotService((ES_Event_t) {PS_TEAM_SELECT,0});
                    }
                    else if (ThisEvent.EventParam == 's')
                    {
                        PostPilotService((ES_Event_t) {PS_FUEL_UPDATE,0});
                    }
                    else if (ThisEvent.EventParam == 'd')
                    {
                        PostPilotSPI((ES_Event_t) {PSPI_DANCE_UPDATE,++NumMoves});
                        NumMoves %= REQ_MOVES; // wrap please
                    }
                    else if (ThisEvent.EventParam == 'i')
                    {
                        PostPilotService((ES_Event_t) {PS_INPUT_MODE, 0});
                    }
                }
                break;

                case (PS_REFUEL):
                {
                    printf("\rTUG OUT OF FUEL\r\n");
                    CurrentState = PilotRefueling;
                    NumMoves = 0;
                    LastJoystickVal[Joy1_X_Posn] = MID;
                    LastJoystickVal[Joy1_Y_Posn] = MID;
                    LastJoystickVal[Joy2_X_Posn] = MID;
                    LastJoystickVal[Joy2_Y_Posn] = MID;
                    JoystickUpdate();
                    PostPilotSPI((ES_Event_t) {PSPI_DANCE_UPDATE, NumMoves});
                    // ES_Timer_InitTimer(SPIUpdate_TIMER,1); // restart timer

                }
                break;

                case (PS_TILT_CHANGE):
                {
                    #ifndef ACCEL_SPHERICAL
                        CurrentAngle = ThisEvent.EventParam;
                        printf("\rWAITING ANGLE: %d\r\n",CurrentAngle);
                    #else // spherical
                        sphericalAngles.combined = ThisEvent.EventParam;
                        printf("Current Phi: %d, Current halfTheta = %d\n\r",
                             sphericalAngles.byAngles.phi, sphericalAngles.byAngles.halfTheta);
                    #endif /* ACCEL_SPHERICAL */
                }
                break;
            }
        }
        break;

        case (PilotRefueling):
        {
            switch (ThisEvent.EventType)
            {
                case (PS_TILT_CHANGE):
                {
                    if (NumMoves > REQ_MOVES)
                    {
                        CurrentState = PilotWaiting;
                        printf("\r REFUEL COMPLETE \r\n");
                        doTheRefuel(); // send the refuel command!
                        NumMoves = 0; //
                        ES_Timer_InitTimer(JOYSTICK_TIMER, JOYSTICK_TIMEOUT); //track joysticks again
                        // ES_Timer_InitTimer(SPIUpdate_TIMER, 1); //track joysticks again
                    }
                    else 
                    {
                        calculateDanceMoves(ThisEvent.EventParam);
                    }
                    
                    PostPilotSPI((ES_Event_t) {PSPI_DANCE_UPDATE, NumMoves});
                    printf("\rNumMoves: %d\r\n", NumMoves);
                }
                break;

                case (ES_Refueling_Override):
                {
                    CurrentState = PilotWaiting;                    
                    printf("\r REFUEL COMPLETE, OVERRIDDEN!\r\n");
                    doTheRefuel(); // send the refuel command!
                    NumMoves = 0; //
                    ES_Timer_InitTimer(JOYSTICK_TIMER, JOYSTICK_TIMEOUT); //track joysticks again
                }
                break;

                case (PS_DRIVE_MODE):
                {
                    if (DriveMode >= Turbo)
                    {
                        DriveMode = Eco;
                    }
                    else
                    {
                        DriveMode++;
                    }
                    PostPilotSPI((ES_Event_t) {PSPI_DRIVE_MODE,DriveModes[DriveMode]});
                }
                break;

                case (PS_INPUT_MODE):
                {
                    InputMode++; // increment
                    InputMode %= NUM_INPUT_MODES; // wraparound
                    ES_Event_t toPost = {PSPI_INPUT_MODE, 0};
                    toPost.EventParam = InputLabels[InputMode][0] << 8 |
                                        InputLabels[InputMode][1];
                    PostPilotSPI(toPost);
                }  
                break;
                
                case (PS_PAIRED):
                {
                    PostPilotSPI((ES_Event_t) {PSPI_PAIR_UNPAIR,
                                 Teams[getTUGlatchedIndex()]});
                }
                break;
                
                case (PS_UNPAIRED):
                {
                    PostPilotSPI((ES_Event_t) {PSPI_PAIR_UNPAIR,Unpaired});
                }
                break;
                
                case (PS_TEAM_SELECT):
                {
                    PostPilotSPI((ES_Event_t) {PSPI_TEAM_SELECT,
                                 Teams[getTUGpairingIndex()]});
                }
                break;
                
                case (PS_FUEL_UPDATE):
                {
                    Fuel = getFuelLevel(); // get from XBee
                    PostPilotSPI((ES_Event_t) {PSPI_FUEL_UPDATE,Fuel});
                }
                break;

                
            }
        }
        break;
    }
    return ReturnEvent;
}

bool joystickForceUpdate_Checker(void) {
    if (2*JOYSTICK_TIMEOUT < (lastJoystickRead - ES_Timer_GetTime())) 
    {
        // been too long, ask for update
        ES_Timer_InitTimer(JOYSTICK_TIMER,1); // restart timer
        return true;
    }
    return false;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static bool InitJoystick()
{
    if (!PortSetup_ConfigureAnalogInputs(J1XPORT,J1XPIN)) return false;
    if (!PortSetup_ConfigureAnalogInputs(J1YPORT,J1YPIN)) return false;
    if (!PortSetup_ConfigureDigitalInputs(J1SWPORT,J1SWPIN)) return false;
    if (!PortSetup_ConfigureAnalogInputs(J2XPORT,J2XPIN)) return false;
    if (!PortSetup_ConfigureAnalogInputs(J2YPORT,J2YPIN)) return false;
    if (!PortSetup_ConfigureDigitalInputs(J2SWPORT,J2SWPIN)) return false;
    ADC_ConfigAutoScan(J1XANPIN | J1YANPIN | J2XANPIN | J2YANPIN, NUMPINS);
    ADC_MultiRead(LastJoystickVal);
    return true;
}

static void JoystickUpdate()
{
    int8_t x; int8_t y; int8_t yaw;
    switch (InputMode)
    {
        case Arcade_2_stick:
        {
            if (LastJoystickVal[Joy1_X_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                float x_float = -CONTROL_MAX * \
                    (range - (LastJoystickVal[Joy1_X_Posn] - MIN)) / range;
                x = (int8_t) x_float;
            }
            else if (LastJoystickVal[Joy1_X_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                float x_float = CONTROL_MAX * \
                    (LastJoystickVal[Joy1_X_Posn] - minval) / range;
                x = (int8_t) x_float;
            }
            else
            {
                x = 0;
            }
            
            if (LastJoystickVal[Joy1_Y_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                float y_float = -CONTROL_MAX * \
                    (range - (LastJoystickVal[Joy1_Y_Posn] - MIN)) / range;
                y = (int8_t) y_float;
            }
            else if (LastJoystickVal[Joy1_Y_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                float y_float = CONTROL_MAX * \
                    (LastJoystickVal[Joy1_Y_Posn] - minval) / range;
                y = (int8_t) y_float;
            }
            else
            {
                y = 0;
            }
            
            if (LastJoystickVal[Joy2_Y_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                float yaw_float = -CONTROL_MAX * \
                    (range - (LastJoystickVal[Joy2_Y_Posn] - MIN)) / range;
                yaw = (int8_t) yaw_float;
            }
            else if (LastJoystickVal[Joy2_Y_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                float yaw_float = CONTROL_MAX * \
                    (LastJoystickVal[Joy2_Y_Posn] - minval) / range;
                yaw = (int8_t) yaw_float;
            }
            else
            {
                yaw = 0;
            }

            // invert controls b/c mounting weirdness
            x = -x;
            y = -y;
            yaw = -yaw;
            break;
        }

        case Arcade_1_stick:
        {
            if (LastJoystickVal[Joy2_X_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                float x_float = -CONTROL_MAX * \
                    (range - (LastJoystickVal[Joy2_X_Posn] - MIN)) / range;
                x = (int8_t) x_float;
            }
            else if (LastJoystickVal[Joy2_X_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                float x_float = CONTROL_MAX * \
                    (LastJoystickVal[Joy2_X_Posn] - minval) / range;
                x = (int8_t) x_float;
            }
            else
            {
                x = 0;
            }

            if (LastJoystickVal[Joy2_Y_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                float yaw_float = -CONTROL_MAX * \
                    (range - (LastJoystickVal[Joy2_Y_Posn] - MIN)) / range;
                yaw = (int8_t) yaw_float;
            }
            else if (LastJoystickVal[Joy2_Y_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                float yaw_float = CONTROL_MAX * \
                    (LastJoystickVal[Joy2_Y_Posn] - minval) / range;
                yaw = (int8_t) yaw_float;
            }
            else
            {
                yaw = 0;
            }

            // invert controls b/c mounting weirdness
            x = -x;
            y = 0; // no y control in this scheme
            yaw = -yaw;
            break;   
        }

        case Tank_2_stick:
        {
            float LTV = 0;
            float RTV = 0; // left and right thrust vectors
            if (LastJoystickVal[Joy1_X_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                LTV = (range - (LastJoystickVal[Joy1_X_Posn] - MIN)) / range;
            }
            else if (LastJoystickVal[Joy1_X_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                LTV = -(LastJoystickVal[Joy1_X_Posn] - minval) / range;
            }
            else
            {
                LTV = 0.0;
            }

            if (LastJoystickVal[Joy2_X_Posn] < MID - TOL)
            {
                float range = MID-TOL-MIN;
                RTV = (range - (LastJoystickVal[Joy2_X_Posn] - MIN)) / range;
            }
            else if (LastJoystickVal[Joy2_X_Posn] > MID + TOL)
            {
                float range = MAX-MID-TOL;
                float minval = MID+TOL;
                RTV = -(LastJoystickVal[Joy2_X_Posn] - minval) / range;
            }
            else
            {
                RTV = 0.0;
            }

            // mix the two values! - old sum/difference method
            #ifndef TANK_DRIVE_ARCADE_EXCEPTIONS
                x   = (int8_t) ((LTV+RTV) * (CONTROL_MAX/2));
                yaw = (int8_t) ((LTV-RTV) * (CONTROL_MAX/2));
            #else // handle edge cases differently
                // re-map to "invert" arcade mode
                if ((LTV > TANK_MAX_THRESHOLD) && (RTV >= TANK_MIN_THRESHOLD))
                { // L is +max, R is positive
                    x   = CONTROL_MAX;
                    yaw = (LTV-RTV) * (CONTROL_MAX);
                }
                else if ((-LTV > TANK_MAX_THRESHOLD) && (RTV <= TANK_MIN_THRESHOLD))
                { // L is -max, R is negative
                    x   = -CONTROL_MAX;
                    yaw = (LTV-RTV) * (CONTROL_MAX);
                }
                else if ((RTV > TANK_MAX_THRESHOLD) && (LTV >= TANK_MIN_THRESHOLD))
                { // R is +max, L is positive
                    x   = CONTROL_MAX;
                    yaw = (LTV-RTV) * (CONTROL_MAX);
                }
                else if ((-RTV > TANK_MAX_THRESHOLD) && (LTV <= TANK_MIN_THRESHOLD))
                { // R is -max, L is negative
                    x   = -CONTROL_MAX;
                    yaw = (LTV-RTV) * (CONTROL_MAX);
                }
                else
                { // standard sum/difference style?
                    x   = (int8_t) ((LTV+RTV) * (CONTROL_MAX/2));
                    yaw = (int8_t) ((LTV-RTV) * (CONTROL_MAX/2));
                }
            #endif /* TANK_DRIVE_ARCADE_EXCEPTIONS */
        
            y   = 0; // no value to send


            // printf("LTV: %5.2f, RTV: %5.2f\n\r", LTV, RTV);
            // printf("LTV: %3.2f, RTV: %3.2f\n\r", LTV, RTV);
            break;
        }

        default:
        {  
            x = 0; y = 0; yaw = 0; // clear
            break;
        }
    }

    // handle Eco/Sport/Turbo switch
    x   = (int8_t) x    * DrivePowerPercents[DriveMode] * 0.01;
    y   = (int8_t) y    * DrivePowerPercents[DriveMode] * 0.01;
    yaw = (int8_t) yaw  * DrivePowerPercents[DriveMode] * 0.01;

    // printf("\rx: %d, y: %d, yaw: %d\r\n",x,y,yaw);
    setControlValues(x,y,yaw);
}

void calculateDanceMoves(uint16_t newAngle)
{
    #ifndef ACCEL_SPHERICAL
        CurrentAngle = newAngle;
        printf("\rCURRENT ANGLE: %d\r\n",CurrentAngle);

        if (NumMoves % 2 == 0 && CurrentAngle > ANGLE_THRESH)
        {
            // for even moves you need to reach a positive angle
            NumMoves++;
        }
        else if (NumMoves % 2 != 0 && CurrentAngle < -ANGLE_THRESH)
        {
            // for odd moves you need to reach a negative angle
            NumMoves++;
        }
    #else // use sphericals
        sphericalAngles.combined = newAngle; // store for easy access
        uint8_t phi = sphericalAngles.byAngles.phi;
        uint8_t halfTheta = sphericalAngles.byAngles.halfTheta;
        printf("Current Phi: %d, Current halfTheta = %d\n\r", phi, halfTheta);

        if (phi > ANGLE_THRESH) 
        { // enough of a bend, check if it's a change!
            if (lastTheta != 0xff) 
            { //not the starting value
                int16_t thetaChange = abs(lastTheta - halfTheta);
                if(thetaChange > (HALF_THETA_MAX/2)) 
                {
                    thetaChange = HALF_THETA_MAX-thetaChange;
                }
                printf("theta change: %d\n\r",thetaChange);

                if ( abs(thetaChange) > THETA_THRESH)
                { // sufficient change!
                    NumMoves++; // increment
                }
            }
            lastTheta = halfTheta; // save for next time
        }
        

    
    #endif /* ACCEL_SPHERICAL */

}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

