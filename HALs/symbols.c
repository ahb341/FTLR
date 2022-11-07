// symbols.c is for printing symbols (sprites, etc.) to the screen
// All symbols are 4x8 px

#include <xc.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "symbols.h"
#include "dbprintf.h"

#define NUMSYMBOLS 8

const uint8_t symbols [NUMSYMBOLS][8] = {               /* SYMBOL NAME  */
    {0x00, 0x07, 0x05, 0x07, 0x02, 0x07, 0x02, 0x05},   /* player       */
    {0x06, 0x09, 0x09, 0x06, 0x06, 0x0f, 0x06, 0x09},   /* bad guy      */
    {0x00, 0x04, 0x0e, 0x15, 0x04, 0x04, 0x04, 0x00},   /* up arrow     */
    {0x00, 0x04, 0x04, 0x04, 0x15, 0x0e, 0x04, 0x00},   /* down arrow   */
    {0x00, 0x00, 0x04, 0x08, 0x1f, 0x08, 0x04, 0x00},   /* left arrow   */
    {0x00, 0x00, 0x04, 0x02, 0x1f, 0x02, 0x04, 0x00},   /* right arrow  */
    {0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff},   /* target lines */
    {0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03}    /* vertical line*/
};


const char *symbolNames[NUMSYMBOLS] = { 
    "player",
    "bad guy",
    "up arrow",
    "down arrow",
    "left arrow",
    "right arrow",
    "target lines",
    "vertical line"
};

const uint32_t bombLines[8] = { 0x3BB87FFF, 0x2AA84001, 0x2AA850D1, 0x2AABDD1D, 
    0x2AABD1D1, 0xEAAA4001, 0xAAA4001, 0xEEE7FFF };

const uint32_t bombExplosionAnimation1[8] = { 0x24000012, 0x92000024, 0x24644A92, 0x92AAAEA4, 0x24CAAA92, 0x92AAAA24, 0x24C44A92, 0x92000024 };
const uint32_t bombExplosionAnimation2[8] = { 0x48000009, 0x24000012, 0x48644A89, 0x24AAAE92, 0x48CAAA89, 0x24AAAA12, 0x48C44A89, 0x24000012 };	
const uint32_t victoryAnimation1[8] = { 0x0, 0x60000014, 0x57200714, 0x55553514, 0x56456674, 0x54651450, 0x67476774, 0x0};
const uint32_t victoryAnimation2[8] = { 0x0, 0x28281414, 0x28281414, 0x28281414, 0x82824141, 0x44442222, 0x38381C1C, 0x0};




///////////////////////////////////////////////////////////////////////////////////
// getSymbolLine is a function that returns the 8-bit hex value of one line of a LED matrix symbol
//  INPUTS:
//      symbolStr - string dictating what symbol to get. See symbolNames array for options. 
//      line - number between 0 (top) and 7 (bottom) of line to get   
//  RETURNS:
//      hex value of corresponding line to print
//  EXAMPLE:
//      uint8_t line2Print = getSymbolLine("player", 0)

uint32_t getSymbolLine(char* symbolStr, uint8_t line) {
    
    uint32_t returnVal = 0;
    
    if ((!strcmp(symbolStr, "bombTimer")) && (line < 8))
        returnVal = bombLines[line];
    else if ((!strcmp(symbolStr, "explode1")) && (line < 8))
        returnVal = bombExplosionAnimation1[line];
    else if ((!strcmp(symbolStr, "explode2")) && (line < 8))
        returnVal = bombExplosionAnimation2[line];
    else if ((!strcmp(symbolStr, "win1")) && (line < 8))
        returnVal = victoryAnimation1[line];
    else if ((!strcmp(symbolStr, "win2")) && (line < 8))
        returnVal = victoryAnimation2[line];
    else 
    {
        // get find index of symbolStr
        uint16_t symbolIdx;
        bool foundSymbol = false;
        for (int i = 0; i < NUMSYMBOLS; i++) 
        {
            if (!strcmp(symbolStr, symbolNames[i])) 
            {
                symbolIdx = i;
                foundSymbol = true;
                break;
            }
        }

        if ((foundSymbol) && line < 8) 
        {
            returnVal = (symbols[symbolIdx][line]);
        } 
        else if (line >= 8) 
        {
            DB_printf("\n\r ERROR (getSymbolLine): line %u is invalid!", line);
        } 
        else 
        {
            DB_printf("\n\r ERROR (getSymbolLine): symbol string %s not found", symbolStr);
        }
    }
    return returnVal;
}

///////////////////////////////////////////////////////////////////////
// getBombWireGuideLine gets the LED row for the bomb wiring task.
//  INPUTS:
//      wireNum - the index of the wire, legal values 1-4
//      nodeNum - the index of the node, legal values 1-8
//      line - the index of the LED display row, legal values 0-7
//  RETURNS:
//      returnVal - 32-bit number where 1 is led on and 0 is LED off


uint32_t getBombWireGuideLine(uint8_t wireNum, uint8_t nodeNum, uint8_t line)
{
    const uint8_t wirePxPosition[] = {28, 20, 11, 3};
    const uint8_t nodePxPosition[] = {30, 26, 22, 18, 13, 9, 5, 1};
    uint32_t returnVal = 0;
    
    if(line == 4)
    {
        // if wire is to the right of node
        if (wirePxPosition[wireNum-1] < nodePxPosition[nodeNum-1])
        {
            for(int i=wirePxPosition[wireNum-1]; i<=nodePxPosition[nodeNum-1]; i++)
            {
                returnVal = returnVal | (0x01 << i);
            }
        }
        else
        {
            for(int i=nodePxPosition[nodeNum-1]; i<=wirePxPosition[wireNum-1]; i++)
            {
                returnVal = returnVal | (0x01 << i);
            }
        }
    }
    // lines 1-3 are vertical from wireNum
    else if(line > 4)
    {
        returnVal = 0x01 << wirePxPosition[wireNum-1];
    }
    else if(line < 4)
    {
        returnVal = 0x01 << nodePxPosition[nodeNum-1];
    }
    
    return returnVal;
}