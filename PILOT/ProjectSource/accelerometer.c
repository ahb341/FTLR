/****************************************************************************
 Module
   Accelerometer.c

 Revision
   1.0.1

 Description
   This is a module for interfacing with the MPU92/65 (similar to
   MPU 9250) accelerometer.

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
#include "terminal.h"

// C Standard Libraries
#include <math.h>

// HALs
#include "MPU9250RegisterMap.h"
#include "../../HALs/pic32_spi_hal.h"

// Project Libraries
#include "PilotSPI.h"
#include "PilotService.h"

// This Module
#include "Accelerometer.h"

/*----------------------------- Module Defines ----------------------------*/
#define OVERSAMPLE_TARGET   16 //how many times to oversample?

#define CALIB_ACCEL_SENSITIVITY   16384 //LSB/g
#define ANGLE_TOLERANCE           1.0 //how much variation before we send event?
#define HALF_THETA_OFFSET         90.0


#define ANGLE_SENSE_METHOD      RPT_SPHERICAL_ANGLES
#define YZ_PLANE_ANGLE          1001
#define XYZ_PLANE_ANGLE         1002
#define RPT_SPHERICAL_ANGLES    1003

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
void sendCmd(uint8_t RegAddr, uint8_t Data);
void requestDataString(uint8_t RegAddr);
bool readByte(uint8_t RegAddr, uint8_t* storeData);
void AccelEventCheckerInit(void);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

typedef enum {Acc_Reset = 0, Acc_Wakeup, Acc_TimeSource, Acc_gryoDLPF,
              Acc_FIFOrate, Acc_gryoFS, Acc_EnAcc, Acc_SetAccSample,
              Acc_FIFOEn, Acc_FIFOsettings, Acc_getBiasData,
              Acc_getFactoryTrimValues, Acc_writeTrimValues,
} InitStep_t;

//steps to read the accelerometer
typedef enum {accXH = 0, accXL, accYH, accYL, accZH, accZL
} Accel_Read_Step_t;
static const uint8_t accelReadRegs[] = {
  ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
  ACCEL_ZOUT_H, ACCEL_ZOUT_L
};

//for bias calibration
static int16_t acc_bias_reg[3] = {0,0,0}; //to put factory trim values
static const uint8_t accelTrimRegs[] = { //regs to read/write for calibration
  XA_OFFSET_H, XA_OFFSET_L, YA_OFFSET_H, YA_OFFSET_L,
  ZA_OFFSET_H, ZA_OFFSET_L,
};
static uint8_t biasBytes[6] = {0,0,0,0,0,0}; //variables to read/write from
static Accel_InD_t initReading = Acc_NotReading; //are we writing or reading?
static uint8_t currentTrimRegister = 0; //init current step

static struct Oversampling
{
    int32_t xRaw;
    int32_t yRaw;
    int32_t zRaw;
    int16_t xClean;
    int16_t yClean;
    int16_t zClean;
    uint8_t numSamples;
    bool externalSignalling;
} Accel_OverSample;

static uint8_t rawData[6]; //create a thing for checking
static struct AccelData {
    int16_t accX;
    int16_t accY;
    int16_t accZ;
} AccelData;



static InitStep_t CurrentInitStep = Acc_Reset;
static uint8_t SideInitStep = 0;
static Accel_Read_Step_t CurrentReadStep = accXH; //start at beginning

static uint16_t Accel_X, Accel_Y, Accel_Z;

//Event checker
static float lastAngle; //what was the last angle we measured?
static bool AccelReady = false; //is the sensor ready for measurement?
static AccDataToSend_t lastAngleStruct; // for event checker, spherical style


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
  Accel_TakeInitUpdateStep

  Description
  Initializes the MPU9250 performing 1 step for each call:
    1: reset device
    2: wake up device
    3: get stable time source
    optional (gyro config??)
    4: gyro dlpf config
    5: set smplrt/fifo rate
    6: gyro full scale range
    7: enable accelerometer axis and set FS range
    8: set accl sample rate
    9: enable FIFO buffer
    10: configure FIFO buffer
****************************************************************************/
bool Accel_TakeInitUpdateStep( void )
{
    static uint8_t rowIndex = 0;
    bool ReturnVal = false;
    
    switch (CurrentInitStep)
    {
        case Acc_Reset:
        {
            printf("\rAcc_Reset\r\n");
            
            //initialize structs
            Accel_OverSample.xRaw = 0;
            Accel_OverSample.yRaw = 0;
            Accel_OverSample.zRaw = 0;
            Accel_OverSample.numSamples = 0;
            
            // First, put it in shutdown to disable all displays
            sendCmd(PWR_MGMT_1, 0x80); //write shutdown command
            
            // move on to next step
            CurrentInitStep++;
        }
        break;
        
        case Acc_Wakeup:
        {
            printf("\rAcc_Wakeup\r\n");
            sendCmd(PWR_MGMT_1,0x00); //pull out of reset
            CurrentInitStep++;
        }
        break;
        
        case Acc_TimeSource:
        {
            printf("\rAcc_TimeSource\r\n");
            sendCmd(PWR_MGMT_1, 0x01); //seletc PLL gyro
            CurrentInitStep++;
        }
        break;
        
        case Acc_gryoDLPF:
        {
            printf("\rAcc_gyroDLPF\r\n");
            sendCmd(MPU_CONFIG, (0 | 6));
            CurrentInitStep++;
        }
        break;
        
        case Acc_FIFOrate:
        {
            printf("\rAcc_FIFOrate\r\n");
            sendCmd(SMPLRT_DIV, 0 | 0);
            CurrentInitStep++;
        }
        break;
        
        case Acc_gryoFS:
        {
            printf("\rAcc_gyroFS\r\n");
            sendCmd(GYRO_CONFIG, (0 | (0b00 <<3) | (0x03)));
            CurrentInitStep++;
        }
        break;
        
        case Acc_EnAcc:
        {
            printf("\rAcc_EnAcc\r\n");
            //set FS output +/- 2g, disable self tests
            uint8_t acclConf = (0b0 | 0b000 << 4 | 0b00 << 3);
            sendCmd(ACCEL_CONFIG, acclConf);
            CurrentInitStep++;
        }
        break;
        
        case Acc_SetAccSample:
        {
            printf("\rAcc_SetAccSample\r\n");
            //set Fchoice to 0, and set DLPF to max setting 4??
            uint8_t acclConf2 = (0 | 0 << 3 | 0b11 << 0);
            sendCmd(ACCEL_CONFIG2, acclConf2);
            CurrentInitStep++;
        }
        break;
        
        case Acc_FIFOEn:
        {
            printf("\rAcc_FIFOEn\r\n");
            sendCmd(USER_CTRL, 0 << 6); //disable for now
            CurrentInitStep++;
        }
        break;
        
        case Acc_FIFOsettings:
        {
            printf("\rAcc_FIFOsettings\r\n");
            sendCmd(FIFO_EN, 0); //just send Accel data
            Accel_OverSample.externalSignalling = 0; //clear this bit now
            initReading = Acc_NotReading; // clear it now, read phase
            CurrentInitStep++;
        }
        break;
        
        case Acc_getBiasData:
        {
            printf("\rAcc_getBiasData\r\n");
            if (0 == Accel_OverSample.externalSignalling)
            {
                SPIOperate_clearBuffer(SPI_SPI1); //ensure we have no extra data
                Accel_requestAccelData(); //ask for data
                initReading = Acc_CalibrationData;
            }
            else
            { //hit data target!
                //normalize gravity
                if (Accel_OverSample.zClean > 0L)
                {
                    Accel_OverSample.zClean -= (float) CALIB_ACCEL_SENSITIVITY;
                }
                else //remove gravity from bias calc
                {
                    Accel_OverSample.zClean += (float) CALIB_ACCEL_SENSITIVITY;
                }
                CurrentInitStep++;
                SPIOperate_clearBuffer(SPI_SPI1); //clear buffer again before next step
                initReading = Acc_NotReading; //prep for next phase
            }
        }
        break;
        
        case Acc_getFactoryTrimValues:
        {
            printf("ACC_INIT: GET FACTORY TRIM VALUES \r\n");
            if (ARRAY_SIZE(accelTrimRegs) > currentTrimRegister)
            { //still have more registers to read
                initReading = Acc_ReadTrimRegisters; //save data!
                requestDataString(accelTrimRegs[currentTrimRegister]);
                currentTrimRegister++; //move to next register
            }
            else
            { //did we hit max array position
                currentTrimRegister = 0; //reset at zero
                initReading = Acc_NotReading; //no longer need to save data

                //handle the data itself!

                //store it into registers
                acc_bias_reg[0] = ((int16_t) biasBytes[0] << 8) | biasBytes[1];
                acc_bias_reg[1] = ((int16_t) biasBytes[2] << 8) | biasBytes[3];
                acc_bias_reg[2] = ((int16_t) biasBytes[4] << 8) | biasBytes[5];

                int16_t acc_bias[] = {
                  (int16_t) Accel_OverSample.xClean,
                  (int16_t) Accel_OverSample.yClean,
                  (int16_t) Accel_OverSample.zClean,
                }; //put into temp array for simple indexing

                printf("\rbias register: %x, %x, %x\r\n",
                acc_bias_reg[0],acc_bias_reg[1],acc_bias_reg[2]);
                printf("\rbiases: %x, %x, %x\r\n",
                acc_bias[0],acc_bias[1],acc_bias[2]);

                int16_t mask_bit[3] = {1,1,1}; //mask bit for acc bias axes
                for (int i = 0; i < 3; i++) 
                {
                  if (acc_bias_reg[i] % 2) 
                  {
                    mask_bit[i] = 0; 
                  }
                  //subtract avg acc bias, scaled to 2048 LSB/g
                  acc_bias_reg[i] -= (int16_t) acc_bias[i] >> 3; 

                  //preserve temp comp bit
                  if (mask_bit[i])
                  {
                    acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i]; 
                  }
                  else
                  {
                    acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;
                  }
                }

                printf("bias registers Calced: %x, %x, %x\n",
                acc_bias_reg[0],acc_bias_reg[1],acc_bias_reg[2]);

                //write new biases into write register buffer like
                biasBytes[0] = (acc_bias_reg[0] >> 8) & 0xFF;
                biasBytes[1] = (acc_bias_reg[0]) & 0xFF;
                biasBytes[2] = (acc_bias_reg[1] >> 8) & 0xFF;
                biasBytes[3] = (acc_bias_reg[1]) & 0xFF;
                biasBytes[4] = (acc_bias_reg[2] >> 8) & 0xFF;
                biasBytes[5] = (acc_bias_reg[2]) & 0xFF;
                CurrentInitStep++; //move to next phase

                //no actual command send, skip the waiting for line to rise
                readTrimRegister(); //read data
            }
        }
        break;
        
        case Acc_writeTrimValues:
        {
            printf("\rAcc_writeTrimValues\r\n");
            sendCmd(accelTrimRegs[currentTrimRegister], biasBytes[currentTrimRegister]);
            currentTrimRegister++; //increment write address
            if (currentTrimRegister >= ARRAY_SIZE(accelTrimRegs))
            { // we have written all the registers!
                currentTrimRegister = 0;  //reset to start
                CurrentInitStep = 0; //reset to start!
                ReturnVal = true; //finished setup
                AccelReady = true; //ready to event check angle
                AccelEventCheckerInit(); //initialize last angle value
            }
        }
        break;
    }
    return ReturnVal;
}

//public function to request data from ALL accel lines
void Accel_requestAccelData(void)
{
    // requestDataString(ACCEL_XOUT_H, 3); //get all accelerometer data
    requestDataString(accelReadRegs[CurrentReadStep]);
}

//public function to store data that's been read
//put all accel data into storage
bool Accel_sequesterAccelData(void)
{
    bool returnVal = false;
    //extract the second byte
    rawData[CurrentReadStep] = SPIOperate_ReadData(SPI_SPI1) & 0x00FF;
    if (CurrentReadStep == accZL) //last step!
    {
        returnVal = true; //ready to exit
        CurrentReadStep = accXH;
        AccelData.accX = rawData[0] << 8 | rawData[1];
        AccelData.accY = rawData[2] << 8 | rawData[3];
        AccelData.accZ = rawData[4] << 8 | rawData[5];


        //store into Oversampler
        Accel_OverSample.xRaw += AccelData.accX;
        Accel_OverSample.yRaw += AccelData.accY;
        Accel_OverSample.zRaw += AccelData.accZ;
        Accel_OverSample.numSamples++; //increment sample counter

        //do we have enough samples to move on?
        if (OVERSAMPLE_TARGET <= Accel_OverSample.numSamples)
        {
            float samples = (float) Accel_OverSample.numSamples; //extract
            Accel_OverSample.xClean = Accel_OverSample.xRaw / samples;
            Accel_OverSample.yClean = Accel_OverSample.yRaw / samples;
            Accel_OverSample.zClean = Accel_OverSample.zRaw / samples;
            Accel_OverSample.xRaw = 0.0; //reset
            Accel_OverSample.yRaw = 0.0; //reset
            Accel_OverSample.zRaw = 0.0; //reset
            Accel_OverSample.numSamples = 0; //reset
            Accel_OverSample.externalSignalling = 1; //set for calib signaling
        }
    }
    else
    {
        CurrentReadStep++;
    }
    return returnVal;
}

Accel_InD_t Accel_Init_needsData(void)
{
    return initReading; //just a getter function
}

void readTrimRegister(void)
{
    //read the data, store into byte array
    biasBytes[currentTrimRegister-1] = SPIOperate_ReadData(SPI_SPI1) & 0x00FF; 
}

int16_t GetX()
{
    return Accel_OverSample.xClean;
}
int16_t GetY()
{
    return Accel_OverSample.yClean;
}
int16_t GetZ()
{
    return Accel_OverSample.zClean;
}

//Event checker to see if the angle has changed
bool AccelAngleChanged(void)
{
    bool returnVal = false; //assume no event triggered
    if (true == AccelReady) //wait until inited
    {
        int16_t aX = Accel_OverSample.xClean;
        int16_t aY = Accel_OverSample.yClean;
        int16_t aZ = Accel_OverSample.zClean;
        //remove any zeros for division errors
        if (0 == aX) aX = 1;
        if (0 == aY) aY = 1;
        if (0 == aZ) aZ = 1;

        #if (ANGLE_SENSE_METHOD == YZ_PLANE_ANGLE) //YZ angle 
          float angle = atan2(aY,aZ)*180.0/M_PI; //curr angle, in degrees
          // char floatString[50];
          // sprintf(floatString,"%f",angle);
          // printf("Angle: %s\n",floatString);
        #elif(ANGLE_SENSE_METHOD == XYZ_PLANE_ANGLE) //handle XYZ angles
            float angle = atan2(aY,sqrt(aZ*aZ + aX*aX))*180.0/M_PI; //curr angle, in degrees
        #elif(ANGLE_SENSE_METHOD == RPT_SPHERICAL_ANGLES)
            static float phi = 0.0;
            static float theta = 0.0;
            // calculate phi, angle off of vertical
            phi = acos(aZ / sqrt(aX*aX + aY*aY + aZ*aZ))* 180/M_PI;

            // calcuate theta, directional angle
            theta = atan2(aX, aY) * 180.0/M_PI; // actual atan2, full 360 degrees

            // store the data!
            static AccDataToSend_t toSend;
            toSend.byAngles.phi         = (uint8_t) phi;
            toSend.byAngles.halfTheta   = (uint8_t) (theta/2.0 + HALF_THETA_OFFSET);
            

        #endif

        #if ((ANGLE_SENSE_METHOD == YZ_PLANE_ANGLE) || (ANGLE_SENSE_METHOD == XYZ_PLANE_ANGLE))
        //did the value change?
        if (ANGLE_TOLERANCE < abs(angle-lastAngle))
        {
            lastAngle = angle; //only update when we detect a change
            int16_t angToSend = (int16_t) angle; //cast it out!!!

            //printf("Angle Change event! Now %d\n",angToSend);
            //post an event!
            ES_Event_t AngleChange = {PS_TILT_CHANGE,-1*angToSend};
            PostPilotService(AngleChange);

            returnVal = true; //event detected!
        }
        #elif (ANGLE_SENSE_METHOD == RPT_SPHERICAL_ANGLES)
        // did value change (either)
        if ((ANGLE_TOLERANCE < abs(toSend.byAngles.phi-lastAngleStruct.byAngles.phi)) || 
            (ANGLE_TOLERANCE < abs(toSend.byAngles.halfTheta-lastAngleStruct.byAngles.halfTheta)))
        {
            // printf("phi: %5.2f, theta: %5.2f, halfTheta: %d \n\r", phi, theta, 
            //     toSend.byAngles.halfTheta);
            lastAngleStruct = toSend; // update
            PostPilotService((ES_Event_t) {PS_TILT_CHANGE, toSend.combined});
        }
        #endif /* ANGLE_SENSE METHOD is YZ or XYZ */
    }
    return returnVal;
}

/***************************************************************************
 private functions
 ***************************************************************************/

void sendCmd(uint8_t RegAddr, uint8_t Data)
{
  uint16_t toSend = (RegAddr << 8) | Data;
  // DB_printf("SPI send: %d\n",toSend);
  SPIOperate_SPI1_Send16(toSend);
}

void requestDataString(uint8_t RegAddr)
{
  // SPIOperate_clearBuffer(SPI_SPI1); //ensure it's ready
  uint16_t toSend = ( (RegAddr | 1<<7) << 8 ); //to second byte
  SPIOperate_SPI1_Send16(toSend); //send the request register
}


//Event checker initial value
void AccelEventCheckerInit(void)
{
  bool returnVal = false; //assume no event triggered
  if (true == AccelReady) //wait until inited
  {
    int16_t aX = Accel_OverSample.xClean;
    int16_t aY = Accel_OverSample.yClean;
    int16_t aZ = Accel_OverSample.zClean;
    //remove any zeros for division errors
    if (0 == aX) aX = 1;
    if (0 == aY) aY = 1;
    if (0 == aZ) aZ = 1;

    #if ((ANGLE_SENSE_METHOD == YZ_PLANE_ANGLE)) //YZ angle 
        lastAngle = atan2(aY,aZ)*180.0/M_PI; //curr angle, in degrees
    #elif(ANGLE_SENSE_METHOD == XYZ_PLANE_ANGLE) //handle XYZ angles
        lastAngle = atan2(aY,sqrt(aZ*aZ + aX*aX))*180.0/M_PI; //curr angle, in degrees
    #elif(ANGLE_SENSE_METHOD == RPT_SPHERICAL_ANGLES)
        // calcuate theta, directional angle
        float ang = atan2(aX, aY)*180.0/M_PI; // actual atan2, full 360 degrees
        
        // calculate phi, angle off of vertical
        float phi = (uint8_t) acos(aZ / sqrt(aX*aX + aY*aY + aZ*aZ))* 180/M_PI;

        // calcuate theta, directional angle
        float theta = (uint8_t) atan2(aX, aY) * 180.0/M_PI; // actual atan2, full 360 degrees


        // store the data!
        lastAngleStruct.byAngles.phi         = (uint8_t) phi;
        lastAngleStruct.byAngles.halfTheta   = (uint8_t) theta/2 + HALF_THETA_OFFSET;
    #endif
  }
}




/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

