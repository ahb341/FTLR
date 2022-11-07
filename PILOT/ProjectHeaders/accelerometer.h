/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef Accelerometer_H
#define Accelerometer_H

#include "ES_Types.h"

// Public Function Prototypes
typedef enum {
  Acc_NotReading = 0,
  Acc_CalibrationData, Acc_ReadTrimRegisters,
} Accel_InD_t;

bool Accel_TakeInitUpdateStep( void ); //updater
void Accel_requestAccelData(void); //pull all accel data
bool Accel_sequesterAccelData(void); //put all accel data into storage
bool AccelAngleChanged(void); //event checker function

Accel_InD_t Accel_Init_needsData(); //function to tell init if we should save data
void readTrimRegister(void); //function to read data during trim reg step
int16_t GetX(void);
int16_t GetY(void);
int16_t GetZ(void);

typedef union {
  uint16_t combined;
  struct 
  {
    uint8_t phi; // off z axis angle, 0-180
    uint8_t halfTheta; // theta/2, ie 0-180 degrees
  } byAngles;
} AccDataToSend_t;
#define HALF_THETA_MAX    180



#endif /* Accelerometer_H */

