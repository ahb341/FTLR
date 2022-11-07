/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef TUG_XBee_H
#define TUG_XBee_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */


// Public Function Prototypes

// bool InitTUGXBee(uint8_t Priority);
// bool PostTUGXBee(ES_Event_t ThisEvent);
// ES_Event_t RunTUGXBee(ES_Event_t ThisEvent);

//configuration 
void ConfigUART(void);

// create messages 
void constructATEcho();
void constructPairingAck(); 
void constructStatusFrame();


#endif /* TUG_XBee_H */

