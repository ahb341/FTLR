/********
 * Debouncing common definitions!
 *
 */

 #ifndef Debouncer_Common_H
 #define Debouncer_Common_H

typedef enum
{
  Debounce_InitPState, low_stable, low_testing, high_stable, high_testing,
} Debouncer_state_t;

#define DEBOUNCE_TIME   25 // [ms] time to ensure we have a solid connection
#define LONG_PRESS_THRESHOLD  250 // [ms] minimum length for "long press"
#define BTN_HOLD_THRESHOLD    1000 // [ms] time for "press and hold"

#endif /* Debouncer_Common_H */
