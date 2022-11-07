/* 
 * File:   DM_Display.h
 * Author: Ed
 *
 * Created on July 15, 2021, 11:54 AM
 */

#ifndef DM_DISPLAY_H
#define	DM_DISPLAY_H
#include "PIC32_SPI_HAL.h"

/****************************************************************************
 Function
 DM_TakeInitDisplayStep
  UI_TakeDisplayUpdateStep, Tmr_TakeDisplayUpdateStep

 Parameter
  None

 Returns
  bool: true when there are no more initialization steps to perform; false
        while there still more steps to be taken

 Description
  Initializes the MAX7219 4-module display performing 1 step for each call:
    First, bring put it in shutdown to disable all displays, return false
    Next fill the display RAM with Zeros to insure blanked, return false
    Then Disable Code B decoding for all digits, return false
    Then, enable scanning for all digits, return false
    The next setup step is to set the brightness to minimum, return false
    Copy our display buffer to the display, return false
    Finally, bring it out of shutdown and return true
   
Example
   while ( false == DM_TakeInitDisplayStep() )
   {} // note this example is for non-event-driven code
****************************************************************************/
bool DM_TakeInitDisplayStep(void);
bool Concon_TakeDisplayUpdateStep( void );
bool Gascon_TakeDisplayUpdateStep( void );

//need to expose SPI setup as well
void DM_SPIsetup(SPI_Module_t WhichModule, uint32_t bitTime_ns);



/****************************************************************************
 Function
  UI_ClearDisplayBuffer

 Parameter
  None

 Returns
 Nothing (void)

 Description
  Clears the contents of the display buffer.
   
Example
   UI_ClearDisplayBuffer();
****************************************************************************/
void Concon_ClearDisplayBuffer( void );
void Gascon_ClearDisplayBuffer( void );

/****************************************************************************
 Function
  DM_ScrollDisplayBuffer

 Parameter
  uint8_t: The number of Columns to scroll

 Returns
 Nothing (void)

 Description
  Scrolls the contents of the display buffer by the indicated number of 
  columns.
   
Example
   DM_ScrollDisplayBuffer(4);
****************************************************************************/
void Concon_ScrollDisplayBuffer( int8_t NumCols2Scroll);
void Gascon_ScrollDisplayBuffer( uint8_t NumCols2Scroll);



/****************************************************************************
 Function
  DM_TakeDisplayUpdateStep

 Parameter
  None

 Returns
  bool: true when all rows have been copied to the display; false otherwise

 Description
  Copies the contents of the display buffer to the MAX7219 controllers 1 row
  per call.
   
Example
   while (false == DM_TakeDisplayUpdateStep())
   {} // note this example is for non-event-driven code
****************************************************************************/
bool DM_TakeDisplayUpdateStep( void );


/****************************************************************************
 Function
  DM_AddChar2DisplayBuffer

 Parameter
  unsigned char: The character to be added to the display
  
 Returns
  Nothing (void)

 Description
  Copies the bitmap data from the font file into the rows of the frame buffer
  at the right-most character position in the buffer  
   
Example
   DM_AddChar2Display('A');
****************************************************************************/
void Concon_AddChar2DisplayBuffer( unsigned char Char2Display);
void Gascon_AddChar2DisplayBuffer( unsigned char Char2Display);
void Concon_AddChar2BufferPos(unsigned char Char2Display, uint8_t RowOrigin, uint8_t ColOrigin);
void Gascon_AddChar2BufferPos(unsigned char Char2Display, uint8_t RowOrigin, uint8_t ColOrigin);
void Concon_AddSymb2DisplayBuffer( char* Symb2Display, uint8_t rowOrigin, int8_t colOrigin);
void Gascon_AddSymb2DisplayBuffer(char* Symb2Display, uint8_t rowOrigin, int8_t colOrigin);

void Concon_AddBit2BufferPos(bool onOff, uint8_t RowOrigin, uint8_t ColOrigin);


/****************************************************************************
 Function
  DM_AddAndScrollBuffer

 Parameter
  unsigned char: The character to be added to the display
  
 Returns
  Nothing (void)

 Description
  just an abstraction to not need to expose the font width
  Auto scrolls, then adds the new character
   
Example
   DM_AddAndScrollBuffer('A');
****************************************************************************/
void Concon_AddAndScrollBuffer(unsigned char Char2Display);
void Gascon_AddAndScrollBuffer(unsigned char Char2Display);

/****************************************************************************
 Function
  UI_PutDataIntoBufferRow

 Parameter
  uint32_t: The new row data to be stored in the display buffer
  uint8_t:  The row (0->7) into which the data will be stored.
  
 Returns
  bool: true for a legal row number; false otherwise

 Description
  Copies the raw data from the Data2Insert parameter into the specified row 
  of the frame buffer 
   
Example
   DM_PutDataInBufferRow(0x00000001, 0);
****************************************************************************/
bool Concon_PutDataIntoBufferRow( uint32_t Data2Insert, uint8_t WhichRow);
bool Gascon_PutDataIntoBufferRow( uint32_t Data2Insert, uint8_t WhichRow);

/****************************************************************************
 Function
  UI_QueryRowData

 Parameter
  uint8_t: The row of the display buffer to be queried
  uint32_t *: pointer to variable to hold the data from the buffer 
  
 Returns
  bool: true for a legal row number; false otherwise

 Description
  copies the contents of the specified row of the frame buffer into the
 location pointed to by pReturnValue
   
Example
   DM_QueryRowData(0,&ReturnedValue);
****************************************************************************/
bool Concon_QueryRowData( uint8_t RowToQuery, uint32_t * pReturnValue);
bool Gascon_QueryRowData( uint8_t RowToQuery, uint32_t * pReturnValue);

/**** Test Harness!!!

functions to implement a basic test harness, and run as we go
********/


#endif	/* DM_DISPLAY_H */

