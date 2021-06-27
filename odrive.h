#include "driver.h"

#if SPINDLE_ODRIVE

#ifndef _SP_ODRIVE_H_
#define _SP_ODRIVE_H_

#define DEFAULT_ODRIVE_CAN_BAUDRATE 	    500000
#define DEFAULT_ODRIVE_4TH_AXIS_NODE_ID     1
#define DEFAULT_ODRIVE_SPINDLE_NODE_ID      0
#define DEFAULT_ODRIVE_CAN_TIMEOUT          150 //ms
#define DEFAULT_ODRIVE_SPINDLE_PPR          4096
#define DEFAULT_ODRIVE_MAX_RPM_ERROR        100
#define DEFAULT_ODRIVE_GEAR_1               40  //Motor Gear
#define DEFAULT_ODRIVE_GEAR_2               36  //Shaft Gear
#define DEFAULT_ODRIVE_SPINDLE_COOLING_AFTER_TIME 		120	// secends cooling after spindle stop
#define DEFAULT_ODRIVE_SPINDLE_COOLING_TEMP_0 	28	// Start cooling temperatur
#define DEFAULT_ODRIVE_SPINDLE_COOLING_TEMP_100	48	// max cooling temperatur
#define DEFAULT_ODRIVE_FAN_VALUE_MIN	    2980// fan min value
#define DEFAULT_ODRIVE_FAN_VALUE_MAX	    3400// fan min value
#define DEFAULT_ODRIVE_LOAD_MAX 			750	// 750 WATT
#define DEFAULT_ODRIVE_SPINDLE_USE_RATIO 	1	// Use Gear1 Gear2 for RPM
#define DEFAULT_ODRIVE_SPINDLE_INVERT_RPM 	0	// Invert velocity value

// extern odrive_settings_type_t odrive_settings; 
// extern status_code_t odrive_spindle_setting (uint_fast16_t param, float value, char *svalue);
// extern void odrive_spindle_settings_restore (void);
// extern void odrive_spindle_settings_report(setting_type_t setting);

extern void odrive_init(void);
// static void sp_ms_event(void);
#endif // SPINDLE_ODRIVE

#endif// _SP_ODRIVE_H_