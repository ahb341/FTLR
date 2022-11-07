/**
 * @file CommStandards.h
 * @author Josh DeWitt (jndewitt@stanford.edu)
 * @brief non header definitions for the 
 *      ME218C 2022 Communication Protocol
 * @version 0.1
 * @date 2022-18-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <xc.h>
#include "CommStandards.h"

const uint16_t TUG_Addresses[Num_TUGs] = 
{
    0x2187,     /* Team 0 */
    0x2086,     /* Team 1 */
    0x2184,     /* Team 2 */
    0x2188,     /* Team 3 */
    0x2085,     /* Team 4 */
    0x2185,     /* Team 5 */
    0x2169,     /* Garbage, don't actually pair */
};