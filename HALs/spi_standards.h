// SPI_Standards contains the 8-bit SPI transmission command types.

// 16-bit commands will be made up of an 8-bit {command} and an 8-bit {parameter}
// e.g. 0x0103 specifies drive forward (01) for 3 cm (03)

#ifndef SPI_Standards
#define SPI_Standards

// Union typedef for storing command and parameter for SPI transmission
typedef union {
	struct {
		uint8_t Parameter; 	// LSB
		uint8_t Command; 	// MSB
	} ByBytes;
	uint16_t Wrapped;
} SPIMessage_t;


//*************************************************//
// UNIVERSAL COMMANDS
// commands sent to leader from either driver or shooter pic
#define RESP_NOTHING 0x0000             // used so follower pics have something to send when SS line is
                                        // dropped, but they don't have anything meaningful to send
#define QUERY_FOLLOWER 0xFFFF           // request info from follower PIC

//*************************************************//
// DRIVING COMMANDS
// commands sent from leader to driver pic
#define CMD_FORWARD_TO_DIST 0x01        // drive forward for a certain distance. Parameter specifies distance in cm.
#define CMD_FORWARD_TO_TAPE 0x02        // drive forward until a horizontal line is crossed (i.e. all 3 tape sensors read line). Paramer is unused.
#define CMD_FORWARD_TO_SWITCH 0x03      // drive forward until a limit switch is triggered. Parameter is unused. 
#define CMD_CW_TO_INDEFINITE 0x04       // rotate cw indefinitely (e.g. for beacon detection). Parameter is unused. 
#define CMD_REVERSE_TO_DIST 0x05        // drive reverse for a certain distance. Parameter specifies distance in cm.
#define CMD_UNDO_LAST_ROT 0x06          // undo last undefinite rotation
#define CMD_CCW_TO_ANGLE 0x07           // rotate ccw for a certain angle. Parameter specifies distance in deg. 
#define CMD_CCW_TO_INDEFINITE 0x08      // rotate ccw indefinitely (e.g. for beacon detection). Parameter is unused. 
#define CMD_CW_TO_ANGLE 0x09            // rotate cw for a certain angle. Parameter specifies distance in deg. 
#define CMD_STOP 0x0B                   // stop driving. Parameter is unused. 

// response sent from driver to leader pic
#define RESP_MOVE_COMPLETE 0x0C         // move was completed. Parameter is unused. 
#define RESP_MOVE_STARTED 0X0D          // move was started (use for indef moves). Param is unused. 
#define RESP_HIT_WALL 0x0E              // while we were driving, we hit the wall (limit switch triggered). Paramter is unused. 

//*************************************************//
// SHOOTING COMMANDS
// commands sent from the leader pic to the shooter pic
#define CMD_PREPARE_TO_BLAST 0x10       // bring firing wheel up to speed
#define CMD_START_BLASTING 0x11         // So anyway, I started blasting. Parameter is unused. 
#define CMD_INCR_FLYWHEEL_SPEED 0x12    // adjustment option to increase flywheel speed. Parameter defines increase in speed in RPM.
#define CMD_DECR_FLYWHEEL_SPEED 0x13    // adjustment option to decrease flywheel speed. Parameter defines decrease in speed in RPM.
#define CMD_INCR_LAUNCH_ANGLE 0x14      // adjustment option to increase launch angle. Parameter defines increase in launch angle in deg. 
#define CMD_DECR_LAUNCH_ANGLE 0x15      // adjustment option to decrease launch angle. Parameter defines decrease in launch angle in deg. 
#define CMD_STOP_BLASTING 0x16          // take the shooter out of shoot mode. Parameter is unused. 
#define CMD_RELOADED 0x17               // tells shooter to reset ball count to 3.

// response sent from the shooter pic to the leader pic
#define RESP_DONE_SHOOTING 0x18         // we've successfully launched all three balls. Parameter is unused. 
#define RESP_MISFIRE 0x19               // the shooter was attempting to launch the ball but did not detect a launch. Parameter is unused. 

#endif /* SPI_Standards */