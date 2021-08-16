// #include "driver.h"
#include "odrive.h"

#if SPINDLE_ODRIVE

#include <stdio.h>
#include "can.h"
#include "pca9685.h"
#include "../grbl/hal.h"
#include "../grbl/override.h"
#include "../grbl/protocol.h"
#include "../grbl/report.h"
#include "../grbl/nvs_buffer.h"
#include "../grbl/state_machine.h"
#include "../grbl/settings.h"
#include "grbl/spindle_sync.h"


//------------------------------------------Odrive Settings----------------------------------

typedef enum odrive_settings_type_t{  // Odrive Settings type
    Settings_Odrive_useRatio = Setting_SettingsMax,
    Settings_Odrive_invert_RPM,
    Settings_Odrive_can_node_id,
    Settings_Odrive_can_baudrate,
    Settings_Odrive_can_timeout,
    Settings_Odrive_encoder_cpr,
    Settings_Odrive_motor_current_limit,
    Settings_Odrive_controller_vel_limit,
    Settings_Odrive_controller_vel_gain,
    Settings_Odrive_controller_vel_integral_gain,
    Settings_Odrive_gear_motor,
    Settings_Odrive_gear_spindle,
	Settings_Odrive_ovr_feed,
	Settings_Odrive_ovr_curit,
    // Settings_Odrive_cooling_after_time,
    // Settings_Odrive_cooling_temp_0,
    // Settings_Odrive_cooling_temp_100,
    Settings_Odrive_Settings_Max
}odrive_settings_type_t;

typedef struct odrive_settings_t{ // Odrive Settings
    bool use_ratio;          // Use Gear Ratio
    bool invert_rpm;         // Motor direction invert
    uint8_t can_node_id;     // Node-ID 
    uint32_t can_baudrate;    // Baudrate CAN_BPS_500*K
    uint16_t can_timeout;      // ms CAN Frame Timeout
    uint16_t encoder_cpr;     // Odrive Encoder cpr
	float motor_current_limit;
	float controller_vel_limit;
	float controller_vel_gain;
	float controller_vel_integrator_gain;
    uint16_t gear_motor;          // Gear at Motor shaft
    uint16_t gear_spindle;          // Gear at Spindle shaft
    // uint16_t cooling_after_time;  // Time in seconds spindle cooling after spindle stop
    // uint16_t fan_temp_0;      // Temperatur 0% = value C°
    // uint16_t fan_temp_100;    // Temperatur 100% = value C°
    // uint16_t load_max;        // Max Load value
	bool ovr_feed;
	float ovr_cur;
} odrive_settings_t;

static nvs_address_t nvs_address;
static settings_changed_ptr settings_changed;
static odrive_settings_t odrive;

static const setting_group_detail_t odrive_groups [] = {
    { Group_Spindle, Group_All, "Spindle ODrive"},
};

static const setting_detail_t odrive_settings[] = {
    { Settings_Odrive_useRatio, Group_Spindle, "Odrive use Ratio", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &odrive.use_ratio, NULL, NULL },
    { Settings_Odrive_invert_RPM, Group_Spindle, "Odrive invert setpoint", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &odrive.invert_rpm, NULL, NULL },
    { Settings_Odrive_can_baudrate, Group_Spindle, "Odrive CAN Baudrate", "125-1000K", Format_Integer, "###0", "125", "1000", Setting_NonCore, &odrive.can_baudrate, NULL, NULL },
    { Settings_Odrive_can_node_id, Group_Spindle, "Odrive Axis Node ID", NULL, Format_Int16, "###0", "0", "65535", Setting_NonCore, &odrive.can_node_id, NULL, NULL },
    { Settings_Odrive_can_timeout, Group_Spindle, "Odrive CAN Timeout", "milliseconds", Format_Int8, "##0", "0", "250", Setting_NonCore, &odrive.can_timeout, NULL, NULL },
    { Settings_Odrive_encoder_cpr, Group_Spindle, "Odrive Encoder CPR", NULL, Format_Int16, "####0", NULL, "65535", Setting_NonCore, &odrive.encoder_cpr, NULL, NULL },
    { Settings_Odrive_motor_current_limit, Group_Spindle, "Odrive Motor cur_limit", NULL, Format_Decimal, "###0.00", NULL, NULL, Setting_NonCore, &odrive.motor_current_limit, NULL, NULL },
    { Settings_Odrive_controller_vel_limit, Group_Spindle, "Odrive Controller vel_limit", "turn/s", Format_Decimal, "#0.0", "12.0", "60.0", Setting_NonCore, &odrive.controller_vel_limit, NULL, NULL },
    { Settings_Odrive_controller_vel_gain, Group_Spindle, "Odrive Controller vel_gain", "Nm/(turn/s)", Format_Decimal, "##0.00", NULL, NULL, Setting_NonCore, &odrive.controller_vel_gain, NULL, NULL },
    { Settings_Odrive_controller_vel_integral_gain, Group_Spindle, "Odrive vel_integrator_gain", "Nm/(turn/s)*s", Format_Decimal, "##0.00", NULL, NULL, Setting_NonCore, &odrive.controller_vel_integrator_gain, NULL, NULL },
    { Settings_Odrive_gear_motor, Group_Spindle, "Odrive Gear on Motor", "T", Format_Int8, "#0", "0", "255", Setting_NonCore, &odrive.gear_motor, NULL, NULL },
    { Settings_Odrive_gear_spindle, Group_Spindle, "Odrive Gear on Spindle", "T", Format_Int8, "#0", "0", "255", Setting_NonCore, &odrive.gear_spindle, NULL, NULL },
    { Settings_Odrive_ovr_feed, Group_Spindle, "Odrive override FEED on err", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &odrive.ovr_feed, NULL, NULL },
    { Settings_Odrive_ovr_curit, Group_Spindle, "Odrive override CURRENT on err", "0=Off", Format_Decimal, "#0.00", "0.0", "5.0", Setting_NonCore, &odrive.ovr_cur, NULL, NULL },
};

static void odrive_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&odrive, sizeof(odrive_settings_t), true);
}

static void odrive_settings_restore (void)
{
    odrive.use_ratio = DEFAULT_ODRIVE_SPINDLE_USE_RATIO;
	odrive.invert_rpm = DEFAULT_ODRIVE_SPINDLE_INVERT_RPM;
	odrive.can_baudrate = (uint32_t)(CAN_BAUD_500K / 1000);
	odrive.can_node_id = DEFAULT_ODRIVE_SPINDLE_NODE_ID;
	odrive.can_timeout = DEFAULT_ODRIVE_CAN_TIMEOUT;
	odrive.encoder_cpr = DEFAULT_ODRIVE_SPINDLE_PPR;
	odrive.motor_current_limit = 60.0f;
	odrive.controller_vel_limit = 120.0f;
	odrive.controller_vel_gain = 0.0f;
	odrive.controller_vel_integrator_gain = 0.0f;
	odrive.gear_motor = DEFAULT_ODRIVE_GEAR_1;
	odrive.gear_spindle = DEFAULT_ODRIVE_GEAR_2;
	odrive.ovr_feed = true;
	odrive.ovr_cur = 0.0f;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&odrive, sizeof(odrive_settings_t), true);
}

static void odrive_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&odrive, nvs_address, sizeof(odrive_settings_t), true) != NVS_TransferResult_OK)
        odrive_settings_restore();
}

static setting_details_t details = {
    .groups = odrive_groups,
    .n_groups = sizeof(odrive_groups) / sizeof(setting_group_detail_t),
    .settings = odrive_settings,
    .n_settings = sizeof(odrive_settings) / sizeof(setting_detail_t),
    .save = odrive_settings_save,
    .load = odrive_settings_load,
    .restore = odrive_settings_restore
};

static setting_details_t *onReportSettings (void)
{
    return &details;
}

//--------------------------------------Odrive defines---------------------------------------

#define report_feedback report_feedback_message((message_code_t)(-1))
#define set_msg_id(cmd,node_id) ((node_id << 5) + cmd)
#define get_node_id(msgID) ((msgID >> 5) & 0x03F)				// Upper 6 bits
#define get_cmd_id(msgID) (msgID & 0x01F)						// Bottom 5 bits

typedef enum Odrive_axis_state{ //Odrive Axisstate
    AXIS_STATE_UNDEFINED = 0,                   //<! will fall through to idle
    AXIS_STATE_IDLE = 1,                        //<! disable PWM and do nothing
    AXIS_STATE_STARTUP_SEQUENCE = 2,            //<! the actual sequence is defined by the config.startup_... flags
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
    AXIS_STATE_MOTOR_CALIBRATION = 4,           //<! run motor calibration
    AXIS_STATE_SENSORLESS_CONTROL = 5,          //<! run sensorless control
    AXIS_STATE_ENCODER_INDEX_SEARCH = 6,        //<! run encoder index search
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,  //<! run encoder offset calibration
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,         //<! run closed loop control
    AXIS_STATE_LOCKIN_SPIN = 9,                 //<! run lockin spin
    AXIS_STATE_ENCODER_DIR_FIND = 10,
    AXIS_STATE_HOMING                = 11,
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
}Odrive_axis_state;

typedef enum Odrive_can_msg{
        MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_SET_CONTROLLER_MODES,
        MSG_SET_INPUT_POS,
        MSG_SET_INPUT_VEL,
        MSG_SET_INPUT_TORQUE,
        MSG_SET_LIMITS,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_INERTIA,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,
        MSG_GET_VBUS_VOLTAGE,
        MSG_CLEAR_ERRORS,
        MSG_GET_TEMPERATURE,
		MSG_SET_VEL_GAINS,
		MSG_RESET_COUNT,
		MSG_ENCODER_INDEX,
        MSG_CO_HEARTBEAT_CMD = 0x700  // CANOpen NMT Heartbeat  SEND
} Odrive_can_msg;

typedef enum odrive_mcode_t{
    Odrive_Set_Current_Limit = 200, //200
    Odrive_Reboot,// = UserMCode_Generic1, //101
    Odrive_Clear_Errors,
    Odrive_Request_State,
	Odrive_Report_Stats,// = UserMCode_Generic2, //102
	Odrive_Set_Fan,
    // UserMCode_Generic3 = 103,
    // UserMCode_Generic4 = 104,
} odrive_mcode_t;

typedef struct axis_parameters_t{
    uint8_t axis_state;
	union error{
    	uint8_t value;
    	struct {
        uint8_t  axis       :1,
                 controller :1,
                 motor      :1,
                 encoder    :1,
                 unassigned	:4;
    	};
	} error;
    bool isAlive;
    int32_t count_pos;
    int32_t count_cpr;
    float pos_estimate;
    float vel_estimate;
    float vbus;
    float ibus;
    float temp_fet;
    float temp_motor;
	float lim_current;
	float lim_vel;
    uint32_t last_heartbeat;
    uint8_t last_state;
}axis_parameters_t;

typedef struct encoder_frame_timer_t{
    volatile uint32_t last_index_us;   // Timer value at last encoder index pulse
    volatile uint32_t last_pulse_us;   // Timer value at last encoder pulse
    volatile uint32_t pulse_length_us; // Last timer tics between spindle encoder pulse interrupts.
	volatile uint32_t frame_time_us;
} encoder_frame_timer_t;

typedef struct {
  uint16_t  active_speed,    // 0-4096 (fullspeed); Speed with enabled Spindle
            idle_speed;      // 0-4096 (fullspeed); Speed after idle period with Spindle disabled
  uint16_t  duration;        // Duration in seconds for the fan to run after Spindle disabled
  bool      auto_mode;       // Default true
  uint16_t  full_speed_temp; // Full speed at this temperatur in auto_mode
  uint8_t 	pwm_channel; 	 // I2C PWM(PCA9685) Fan channel 0-16
} Fan_settings_t;

typedef struct fan_t{ //Fan
    enum fan_state_t{
		FAN_IDLE,
		FAN_ACTIVE,
		FAN_START,
		FAN_STOP,
		FAN_WAIT_STOP,
    	FAN_DURATION
	}state;
	uint16_t speed;
    uint8_t last_state;
    uint16_t last_temperatur;
	Fan_settings_t *settings;
}Fan_t;

#define FAN_SPEED_ACTIVE 	1400
#define FAN_SPEED_IDLE     	0
#define FAN_IDLE_TIME     	60
#define FAN_FULL_SPEED_TEMP 50
#define FAN_PWM_CHANNEL			0

static Fan_settings_t Fan_settings = {
  FAN_SPEED_ACTIVE,
  FAN_SPEED_IDLE,
  FAN_IDLE_TIME,
  true,
  FAN_FULL_SPEED_TEMP,
  FAN_PWM_CHANNEL
};

typedef union flag_t{
    struct {
        uint8_t timeout          :1,
                got_error        :1,
                got_rpm_diff     :1,
                ovr_active    	 :1,
                wait_off      	 :1,
                wait_accel    	 :1,
                motor_error      :1,
                temperatur_error :1;
    };
} flag_t;

typedef struct rpm_diff_t{
	enum rpm_diff_state_t{
		DIFF_IDLE,
		DIFF_OVR_ON,
		DIFF_STATE_HOLD,
		DIFF_RESTORE,
	} state;
	uint32_t t_start;
	uint32_t t_last;

	uint_fast8_t f_start;
	float 		 c_start;

	float max_c;
	float max_diff;
}rpm_diff_t;

typedef struct periodic_frame_t {
    volatile bool active;
	uint8_t cmd_id;
    uint16_t interval;
	uint32_t last;
}periodic_frame_t; 

static periodic_frame_t frames_periodic[] = {
        {On, MSG_GET_VBUS_VOLTAGE, 			150, 0 },
        // {Off, MSG_GET_ENCODER_ESTIMATES,	5, 0},
        {On, MSG_GET_ENCODER_COUNT,  		20, 0},
        {On, MSG_GET_TEMPERATURE, 			100, 0}
};

static axis_parameters_t sp_axis ={0};
static Fan_t fan = {.settings=&Fan_settings};
static flag_t flag = {0};
static rpm_diff_t diff = {0};

static float rpm_diff = 0.0f, rpm_tol;
static spindle_state_t sp_state = {0};
static spindle_data_t sp_data = {0};
static CANListener odrv_listener = {0};

static spindle_encoder_t spindle_encoder;
static spindle_sync_t spindle_tracker;
static spindle_encoder_counter_t encoder;
static encoder_frame_timer_t encoder_timer;

static on_execute_realtime_ptr on_execute_realtime;
static on_realtime_report_ptr on_realtime_report;
static on_unknown_feedback_message_ptr on_unknown_feedback_message;
static on_program_completed_ptr on_program_completed;
static driver_reset_ptr driver_reset;
spindle_data_t *spindleGetData (spindle_data_request_t request);
static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;
static char msg_debug[128] = {0};
static char msg_feedback[128] = {0};
static char msg_warning[128] = {0};
static void spindle_rpm_diff(sys_state_t state);
static CAN_sync_message_t* s_msg = NULL;
static bool init_ok = false;
//-------------------------------Odrive CAN Connection Commands------------------------

void report_FrameData(CAN_message_t *frame){
	char msgString[80];
	sprintf(msgString, "[Frame ID:%02lu DLC:%01u Data: %02u %02u %02u %02u %02u %02u %02u %02u]",
						frame->id,frame->len,
						frame->buf[0],frame->buf[1],frame->buf[2],frame->buf[3],
						frame->buf[4],frame->buf[5],frame->buf[6],frame->buf[7]
						);
	report_message(msgString,-1);
}

int odrive_set_state(uint8_t axis_state, bool block){
	if (!sp_axis.isAlive) return -1;
    CAN_message_t out = {0};
	out.id = set_msg_id(MSG_SET_AXIS_REQUESTED_STATE,odrive.can_node_id);
	out.len = 4;
	out.low = axis_state;
	// out.buf[0] = axis_state;
	// out.buf[1] = 0;
	// out.buf[2] = 0;
	// out.buf[3] = 0;
	// out.high = 0;
	return canbus_write_blocking(&out, block);
}

void odrive_get_parameter(uint8_t cmd, bool block){
	if (!sp_axis.isAlive)
		return;
    CAN_message_t out = {0};
	out.flags.remote = 1;
	out.len = 0;
	out.id = set_msg_id(cmd,odrive.can_node_id);
	// if (block)
		canbus_write_blocking(&out,block);
	// else
		// canbus_write(&out);
}

void odrive_set_input_vel(float vel, bool block){
	if (!sp_axis.isAlive){
		// sprintf(msg_warning,"INPUT VEL fail! id:%d vel:%.2f odrvAlive:%d can:%d",odrive.can_node_id,vel,sp_axis.isAlive,(uint8_t)canbus_connected());
		// report_message(msg_warning,Message_Warning);
		// memset(&msg_warning,0,sizeof(msg_warning));
		return;
	}
	// if (msg_debug[0] == '\0')
		// sprintf(msg_debug,"spindle_update rpm=%.0f vel=%.2f", sp_data.rpm_programmed, vel);
		
	CAN_message_t out = {0};
	out.id = set_msg_id(MSG_SET_INPUT_VEL,odrive.can_node_id);
	out.len = 4;
	uint32_t u_vel = 0;
	memcpy(&u_vel,&vel,sizeof(vel));
	out.low = u_vel;
  	out.high = 0;
	canbus_write_blocking(&out, block);
}

int odrive_set_limits(float vel, float cur, bool block){
	if (!sp_axis.isAlive){
		// sprintf(msg_feedback,"SET LIMITS fail! id:%d odrvAlive:%d can:%d",odrive.can_node_id,sp_axis.isAlive,(uint8_t)canbus_connected());
		// report_feedback;
		return 0;
	}    
	CAN_message_t out = {0};
	out.id = set_msg_id(MSG_SET_LIMITS,odrive.can_node_id);
	out.len = 8;
	uint32_t u_vel = 0,u_cur = 0;
	memcpy(&u_vel,&vel,sizeof(vel));
	memcpy(&u_cur,&cur,sizeof(cur));
	out.low = u_vel;
  	out.high = u_cur;
	return canbus_write_blocking(&out,block);
}

int odrive_clear_errors(bool block){
	if (!sp_axis.isAlive) return -1;
    CAN_message_t out = {0};
	out.len = 0;
	out.flags.remote = 1;
	out.id = set_msg_id(MSG_CLEAR_ERRORS,odrive.can_node_id);
	return canbus_write_blocking(&out, block);
}

//-------------------------------CAN Callbacks-----------------------------------------

void cb_gotFrame(CAN_message_t *frame, int mb){
  if (get_node_id(frame->id) != odrive.can_node_id) 
  	return;
  uint8_t cmd = get_cmd_id(frame->id);
  
  switch (cmd){
  	case MSG_ODRIVE_HEARTBEAT:{
		uint8_t lastState = sp_axis.axis_state;
		memcpy(&sp_axis.error, &frame->buf[0], sizeof(uint8_t));
    	memcpy(&sp_axis.axis_state, &frame->buf[1], sizeof(uint8_t));
		if (sp_axis.axis_state != lastState)
			sp_axis.last_state = lastState;
		sp_state.on = sp_axis.axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL ? On : Off;
		flag.got_error = sp_axis.error.value == 0 ? Off : On;
		sp_axis.last_heartbeat = hal.get_elapsed_ticks();
		if (!sp_axis.isAlive){
			sp_axis.isAlive = On;
			hal.spindle.get_data = *spindleGetData;
			report_message("CAN connected",Message_Info);
			// odrive_set_limits(odrive.controller_vel_limit,odrive.motor_current_limit,false);
		}
    	// bool a_err = sp_axis.error.axis, c_err = sp_axis.error.controller, m_err = sp_axis.error.motor, e_err = sp_axis.error.encoder;
		// sprintf(msg_feedback,"Frame hBeat state:%u error:%u ax:%u co:%u mo:%u en:%u buf: %02x%02x%02x%02x",
		// 			sp_axis.axis_state,sp_axis.error.value,a_err,c_err,m_err,e_err,
		// 			frame->buf[0],frame->buf[1],frame->buf[2],frame->buf[3]);
		// report_feedback;
		// sprintf(feedback_msg,"Frame HEARTBEAT can:%d alive:%d state:%d lastState:%d last:%d",canbus_connected(),sp_axis.isAlive,sp_axis.axis_state,sp_axis.last_state,sp_axis.last_heartbeat);
		// report_feedback;
  	    break;}
  	case MSG_GET_VBUS_VOLTAGE:{
		uint32_t data = 0UL;
		memcpy(&data,&frame->buf,sizeof(data));
		int16_t i_vbus = data >> 16| data;
		int16_t i_ibus = data >> 24 | data >> 16;
		sp_axis.vbus = (float)(i_vbus) / 10.0f;
		sp_axis.ibus = (float)(i_ibus) / 1000.0f;
		// sprintf(msg_feedback,"Frame VBUS vBus:%.1f iBus:%.3f v:%u i:%u len:%u buf: %02x%02x%02x%02x",sp_axis.vbus,sp_axis.ibus,i_vbus,i_ibus,
		// 		frame->len,frame->buf[0],frame->buf[1],frame->buf[2],frame->buf[3]);
		// report_feedback;
  	  	break;}
  	case MSG_GET_TEMPERATURE:{
		memcpy(&sp_axis.temp_fet, &frame->low, sizeof(float));
		memcpy(&sp_axis.temp_motor, &frame->high, sizeof(float));
  	  	break;}
  	case MSG_SET_LIMITS:{
		memcpy(&sp_axis.lim_vel, &frame->low, sizeof(float));
		memcpy(&sp_axis.lim_current, &frame->high, sizeof(float));
  	  	break;}
  	case MSG_GET_ENCODER_COUNT:{
		// uint32_t now_us = micros();
		// spindle_encoder.timer.last_pulse = micros();
		// encoder_timer.frame_time_us = now_us - encoder_timer.last_pulse_us;
		// encoder_timer.last_pulse_us = now_us;
		spindle_encoder.counter.last_index = sp_axis.count_pos / odrive.encoder_cpr;
		spindle_encoder.counter.last_count = sp_axis.count_cpr;
  	  	memcpy(&sp_axis.count_pos, &frame->low, sizeof(sp_axis.count_pos));
  	  	memcpy(&sp_axis.count_cpr, &frame->high, sizeof(sp_axis.count_cpr));
		sp_data.angular_position = (360.0f/odrive.encoder_cpr) * sp_axis.count_cpr;
		spindle_encoder.counter.index_count = sp_axis.count_pos / odrive.encoder_cpr;
		spindle_encoder.counter.pulse_count = sp_axis.count_pos;
		// sp_data.angular_position = (float)encoder.index_count +
                    // ((float)(sp_data.pulse_count) * (1.0f / odrive.encoder_cpr));
                            //  (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
  	  	// static uint32_t next_print = 0;
		// static int_fast32_t last_cpr = {0}, last_pos = {0};
		// if ( /*next_print <= hal.get_elapsed_ticks() || */ last_pos != (int_fast32_t)sp_axis.count_pos && sp_axis.axis_state == AXIS_STATE_IDLE){
			// last_cpr = (int_fast32_t)sp_axis.count_cpr;
			// last_pos = (int_fast32_t)sp_axis.count_pos;
			// next_print = hal.get_elapsed_ticks() + 1000;
			// if (!msg_debug[0]){
				// sprintf(msg_debug,"cpr:%04i pos:%i", 
					// last_cpr, last_pos); // sp_data.index_count, encoder_timer.frame_time_us);
				// report_message(msg_debug,Message_Info);
			// }
			// encoder_timer.frame_time_us = -1;
		// }
  	  	break;}
  	case MSG_GET_ENCODER_ESTIMATES:{
  	    memcpy(&sp_axis.pos_estimate, &frame->low, sizeof(sp_axis.pos_estimate));
    	memcpy(&sp_axis.vel_estimate, &frame->high, sizeof(sp_axis.vel_estimate));
		sp_data.rpm = ((sp_axis.vel_estimate * (sp_axis.vel_estimate < 0.0f ? -60.0f : 60.0f)) /  
				odrive.gear_spindle) * odrive.gear_motor;
		bool atSpeed = settings.spindle.at_speed_tolerance <= 0.0f ? On : 
				(sp_data.rpm >= sp_data.rpm_low_limit && sp_data.rpm <= sp_data.rpm_high_limit);
		if(flag.wait_accel){
			flag.wait_accel = !atSpeed;
		}
		sp_state.at_speed = !flag.wait_accel && !flag.wait_off ? atSpeed : Off;

		rpm_diff = (!atSpeed && !flag.wait_accel && !flag.wait_off && sp_data.rpm_programmed > 0.0f) || rpm_diff > 0.0f ?  
					sp_data.rpm_programmed - min(sp_data.rpm,sp_data.rpm_programmed) : 0.0f;

		if (!flag.got_rpm_diff && state_get() == STATE_CYCLE)
			protocol_enqueue_rt_command(spindle_rpm_diff);
  	    // sprintf(feedback_msg,"Frame ENC EST pos:%.2f vel:%.4f",sp_axis.pos_estimate,sp_axis.vel_estimate);
		// report_feedback;
  	    break;}
	case MSG_ENCODER_INDEX:{
		digitalToggleFast(LED_BUILTIN);}
  	default:
  	  	break;
  	}
}

//-------------------------------------------------------------------------------------

uint16_t map_fan_value(){
	uint16_t temp = max(fan.last_temperatur,fan.settings->full_speed_temp);
	float seq = (float)(4096.0f - fan.settings->active_speed) / (float)(fan.settings->full_speed_temp - 25.0f);
	return (uint16_t)(seq * (temp - 25));
}

void handle_cooling(void){
	bool handle = false;
	static uint32_t aftercooling_start, next_check;
	static uint16_t last_speed;
	uint32_t ms = hal.get_elapsed_ticks();
	if (ms < next_check)
		return;
	next_check = ms + 2000;
	if (fan.last_temperatur != (uint16_t)sp_axis.temp_motor){
		fan.last_temperatur = (uint16_t)sp_axis.temp_motor;
		handle = On;
	}
	else if (fan.last_state != fan.state){
		fan.last_state = fan.state;
		handle = On;
	}
	if (!handle) return;
	uint8_t state = fan.state;
	switch (state)
	{
	case FAN_IDLE:{
		fan.speed = 0;
		break;}
	case FAN_START:{
		fan.speed = map_fan_value();
		aftercooling_start = 0;
		fan.state = FAN_ACTIVE;
		break;}
	case FAN_STOP:{
		aftercooling_start = ms;//1000 * fan.settings->duration;
		fan.state = FAN_WAIT_STOP;
		break;}
	case FAN_WAIT_STOP:{
		bool time_over = ms >= aftercooling_start + (fan.settings->duration * 1000);
		if (time_over || sp_axis.temp_motor < 28.0f) {
			aftercooling_start = 0;
			fan.state = FAN_IDLE;
			fan.speed = 0;
		}
		else 
			fan.speed = map_fan_value();
		break;}
	case FAN_ACTIVE:{
		fan.speed = map_fan_value();
		break;}
	default:
		break;
	}
	if (last_speed != fan.speed){
		last_speed = fan.speed;
		pca9685_pwm_set_duty(fan.settings->pwm_channel,fan.speed);
	}
}

void spindle_rpm_diff(sys_state_t state){
	uint32_t ms = hal.get_elapsed_ticks();
	// static uint16_t org_enc_cound_period = {0};
	static uint32_t last_feed = {0};
	static uint32_t last_cur = {0};
	bool got_diff = rpm_diff > 0.0f && !flag.wait_off;
	bool check = 0;
	bool timeout = 0;
	bool ovr_cur_active = odrive.ovr_cur > 0.0f;
	bool ovr_feed_active = odrive.ovr_feed;
	UNUSED(state);
	
	if (got_diff || diff.state != DIFF_IDLE){

		switch (diff.state)
		{
		case DIFF_IDLE:
			if (ovr_cur_active && sp_axis.lim_current == 0.0f){
				if (sp_axis.lim_current == 0.0f){
					odrive_get_parameter(MSG_SET_LIMITS,false);
				}
				return;
			}
			diff.f_start = ovr_feed_active ? (uint_fast8_t)sys.override.feed_rate : 0;
			diff.c_start = ovr_cur_active && !(sp_axis.lim_current == 0.0f) ? sp_axis.lim_current : 0.0f;
			diff.t_start = diff.t_last = ms - 20;
			// org_enc_cound_period = encoder_estimates_frame->interval;
			// encoder_estimates_frame->interval = 5;
			// encoder_estimates_frame->last = ms - 10;
			// sprintf(msg_feedback,"new ovr state=%u start=%u act=%u diff=%.f c_on=%u start_c=%.1f act_c=%.1f", 
			// 	(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,ovr_cur_active,diff.c_start,sp_axis.lim_current);
			// report_feedback;
			diff.state = DIFF_OVR_ON;
			
		case DIFF_OVR_ON:{
			if (ms >= (diff.t_last + 150)){
				diff.state = DIFF_RESTORE;
				return;
			}

			check = ms >= diff.t_last + 15;
			timeout = ms >= diff.t_start + 800;
			if (!check || !got_diff)
				return;
			
			if (timeout){
				diff.state = DIFF_STATE_HOLD;
				sprintf(msg_feedback,"Still RPM diff=%3.0f Stoping FEED",rpm_diff);
				report_feedback;
				protocol_enqueue_realtime_command(CMD_FEED_HOLD);
				system_set_exec_state_flag(EXEC_FEED_HOLD);
				break;
			}

			if(diff.f_start){
				enqueue_feed_override(CMD_OVERRIDE_FEED_FINE_MINUS);
			}

			if (diff.c_start){
				float add_val = odrive.ovr_cur * sp_data.rpm_programmed < 1200.0 ? 1.5f : 1.25f;
				odrive_set_limits(sp_axis.lim_vel, (sp_axis.lim_current + add_val) > 60.0f ? 60.0f : sp_axis.lim_current + add_val, true);
				// sprintf(msg_feedback,"plus ovr=%u f_s=%u f=%u d=%.f c_s=%.1f c=%.1f c_n=%.1f", 
				// 	(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,diff.c_start,sp_axis.lim_current,sp_axis.lim_current + add_val);
				// report_feedback;
			}

			// sprintf(msg_feedback,"ovr state=%u start=%u act=%u diff=%.f atS=%u cur=%.1f", 
			// 	(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,sp_state.at_speed,sp_axis.lim_current);
			// report_feedback;
			diff.t_last = ms;

			break;}
		case DIFF_STATE_HOLD:{
			timeout = ms >= diff.t_last + 500;
			if(timeout && got_diff){
				sprintf(msg_feedback,"Still RPM diff - stoping Spindle...");
				report_feedback;
				hal.spindle.set_state((spindle_state_t){0}, 0.0f);
				system_set_exec_state_flag(EXEC_STATUS_REPORT);
				diff.state = DIFF_RESTORE;
				diff.t_last = ms;
				return;
			}
			else if(!got_diff){
				diff.state = DIFF_RESTORE;
				diff.t_last = ms;
				return;
			}
			break;}
		case DIFF_RESTORE:{
			check = ms >= (diff.t_last + 25);
			if (got_diff){
				diff.t_start = diff.t_last = ms ;
				diff.state = DIFF_OVR_ON;
				return;
			}

			if (!check)
				return;
				
			// diff.t_last = ms;
			// float current_now = sp_axis.lim_current;
			bool restore_feed =  (uint_fast8_t)sys.override.feed_rate != diff.f_start;
			bool restore_current = ovr_cur_active && (sp_axis.lim_current != diff.c_start);

			if (!restore_current && !restore_feed){
				// encoder_count_frame->interval = org_enc_cound_period;
				// org_enc_cound_period = 0;
				// encoder_count_frame->last = ms;
				diff = (rpm_diff_t){0};
				// diff.state = DIFF_IDLE;
				// sprintf(msg_feedback,"end ovr state=%u start=%u act=%u diff=%.f start_c=%.1f act_c=%.1f", 
				// 	(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,diff.c_start,sp_axis.lim_current);
				// report_feedback;
				break;
			}

			if (restore_feed && ms >= (last_feed + 20)){
				enqueue_feed_override(CMD_OVERRIDE_FEED_FINE_PLUS);
				last_feed = ms;
			}

			if (restore_current && ms >= (last_cur + 50)){
				last_cur = ms;
				float add_val = sp_axis.lim_current - (odrive.ovr_cur * (sp_data.rpm_programmed > 1200.0 ? 1.5f : 1.25f));
				float new_val = add_val < diff.c_start ? diff.c_start : add_val;
				odrive_set_limits(sp_axis.lim_vel,new_val,false);
				// sprintf(msg_feedback,"minus ovr=%u f_s=%u f_n=%u d=%.f r_f=%u r_c=%u c_s=%.1f c=%.1f c_n=%.1f", 
				// 	(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,restore_feed,restore_current,diff.c_start,sp_axis.lim_current,new_val);
				// report_feedback;
			}

			// else {
			// 	sprintf(msg_feedback,"ovr state=%u start=%u act=%u diff=%.f atS=%u start=%.1f cur=%.1f", 
			// 		(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,sp_state.at_speed,diff.c_start,sp_axis.lim_current);
			// 	report_feedback;
			// }
			diff.t_last = ms;
			break;}
		}
	}

	flag.got_rpm_diff = diff.state == DIFF_IDLE ? Off : On;
}

static void spindle_update(void){
	float out_vel = 0.0f;
	bool invert = (!sp_data.state_programmed.ccw && odrive.invert_rpm) || (sp_data.state_programmed.ccw && !odrive.invert_rpm);
	out_vel = ((((sp_data.rpm_programmed) / 60.0f)/odrive.gear_motor) * odrive.gear_spindle);
	out_vel = invert ? -1.0f * out_vel : 1.0f * out_vel;
	odrive_set_input_vel(out_vel,true);
}

static void spindle_on(sys_state_t state){
	if (!sp_axis.isAlive) return;
	
	fan.state = FAN_START;
	// if (odrive.motor_current_limit > 0.0f && sp_axis.lim_current != odrive.motor_current_limit){
	// 	sp_axis.lim_current = 0.0f;
	// 	odrive_set_limits(sp_axis.lim_vel, odrive.motor_current_limit,true);
	// }
	
	if (sp_axis.axis_state == AXIS_STATE_IDLE){
		odrive_set_state(AXIS_STATE_CLOSED_LOOP_CONTROL,true);
	}

	// else {
	// 	static uint32_t _timeout = 0;
	// 	_timeout = !_timeout ? hal.get_elapsed_ticks() : _timeout;

	// }
}

static void spindle_off(sys_state_t state){
	if(!sp_axis.isAlive) return;
	
	if(!flag.wait_off){
		flag.wait_off = On;
		fan.state = FAN_STOP;
	}
	bool running = sp_data.rpm > 20.0f;
	if (!running){
		flag.wait_off = Off;
		if (sp_axis.axis_state != AXIS_STATE_STARTUP_SEQUENCE && sp_axis.axis_state != AXIS_STATE_IDLE)
			odrive_set_state(AXIS_STATE_IDLE,false);
	}
	else
		protocol_enqueue_rt_command(spindle_off);
}

static void spindleUpdateRPM(float rpm)
{
	// sp_state.at_speed = Off;
	flag.wait_accel = rpm > sp_data.rpm && sp_axis.isAlive ? On : Off;
    sp_data.rpm_programmed = rpm;
	// flag.wait_off = rpm == 0.0f && sp_axis.isAlive ? On : Off;
	if(settings.spindle.at_speed_tolerance > 0.0f) {
		rpm_tol = rpm > 0.0f && rpm <= 250.0f ? (250.0f / rpm) * settings.spindle.at_speed_tolerance : settings.spindle.at_speed_tolerance;
        sp_data.rpm_low_limit = rpm == 0.0f ? 0.0f : rpm / (1.0f + rpm_tol);
        sp_data.rpm_high_limit = rpm == 0.0f ? 0.0f : rpm * (1.0f + rpm_tol);
	}
	// if(sp_axis.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //     float delay = 0.0f;
    //     while(sp_axis.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //         delay_sec(0.1f, DelayMode_Dwell);
    //         delay += 0.1f;
    //         if(ABORTED)
    //             return;
    //         if(delay >= SAFETY_DOOR_SPINDLE_DELAY) {
    //             system_raise_alarm(Alarm_Spindle);
    //             return;
    //         }
    //     }
    // }

	// if(rpm > 0.0f && sp_axis.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //     float delay = 0.0f;
    //     while(sp_axis.axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //         delay_sec(0.1f, DelayMode_Dwell);
    //         delay += 0.1f;
    //         if(ABORTED)
    //             break;
    //         if(delay >= SAFETY_DOOR_SPINDLE_DELAY) {
    //             system_raise_alarm(Alarm_Spindle);
    //             break;
    //         }
    //     }
    // }
	// if (sp_axis.isAlive && sp_axis.axis_state == AXIS_STATE_UNDEFINED)
		// report_error_details();
  	spindle_update();
}

static void spindleSetState(spindle_state_t state, float rpm)
{
	sp_data.state_programmed = state;
	sp_data.rpm_programmed = rpm;
	sp_state.ccw = state.ccw;
	// sp_state.at_speed = Off;
	if (!sp_axis.isAlive){
		sp_data.rpm = rpm;
		sp_state.on = state.on;
	}
	else {
  		if (!state.on || (state.on && rpm == 0.0f)) {
			spindleUpdateRPM(rpm);
    		spindle_off(state_get());
  		} else {
    		spindle_on(state_get());
    		spindleUpdateRPM(rpm);
  		}
	}
}

static spindle_state_t spindleGetState (void)
{
  spindle_state_t state = {0};
  if (sp_axis.isAlive){
	  return sp_state;
	// state.on = sp_state.on;
	// state.ccw = sp_state.ccw;
	// state.at_speed = sp_state.at_speed;
  }
  else{
	state.on = sp_data.state_programmed.on;
	state.ccw = sp_data.state_programmed.ccw;
	state.at_speed = On;
  }
  return state;
}

spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    // bool stopped;
    // uint32_t frame_length_us;
    // spindle_encoder_counter_t encoder;

    // __disable_irq();

    // memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    // pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    // rpm_timer_delta = GPT1_CNT - spindle_encoder.timer.last_pulse;

    // __enable_irq();

    // // If no (4) spindle pulses during last 250 ms assume RPM is 0
    // if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
    //     sp_data.rpm = 0.0f;
    //     rpm_timer_delta = (GPT2_CNT - spindle_encoder.counter.last_count) * pulse_length;
    // }

    switch(request) {

        case SpindleData_Counters:
            // sp_data.pulse_count = GPT2_CNT;
            // sp_data.index_count = spindle_encoder.index_count;
            // sp_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            // if(!stopped)
                // sp_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AngularPosition:
			odrive_get_parameter(set_msg_id(MSG_GET_ENCODER_COUNT,odrive.can_node_id),true);
			// encoder.last_count = micros();
            // int32_t d = encoder.last_count - encoder.last_index;
            // sp_data.angular_position = (float)encoder.index_count +
            //         ((float)(d) +
            //                  (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
            //                     spindle_encoder.pulse_distance;
            break;
    }

    return &sp_data;
}

static void spindleDataReset (void)
{
    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = 0;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;
	
	encoder_timer.frame_time_us =
	encoder_timer.last_index_us =
	encoder_timer.last_pulse_us =
	encoder_timer.pulse_length_us = 0;
}

//-------------------------------------Report--Feedback--Realtime---------------------

void sp_feedback_message(stream_write_ptr stream_write){
	if(on_unknown_feedback_message)
        on_unknown_feedback_message(stream_write);
	
	if (msg_feedback[0] != '\0'){
		stream_write(msg_feedback);
		memset(msg_feedback,0,sizeof(msg_feedback));
	}
}

void sp_rt_report(stream_write_ptr stream_write, report_tracking_flags_t report){
	// static report_tracking_flags_t last_report = {0};
	static char rt_report_msg[30] = {0};
	static uint8_t sp_counter = REPORT_WCO_REFRESH_IDLE_COUNT / 2;

	if (sp_counter > 0 && !sys.report.wco)
        sp_counter--;
    else
        sp_counter = REPORT_WCO_REFRESH_IDLE_COUNT / 2;
	
	if(on_realtime_report)
        on_realtime_report(stream_write, report);

	if (sp_axis.isAlive && !sp_counter){
		stream_write("|ODRV:");
    	sprintf(rt_report_msg,"%2.1f,%4.3f,%3.1f",sp_axis.temp_motor > 0.0f ? sp_axis.temp_motor : sp_axis.temp_fet,
												sp_axis.ibus, sp_axis.ibus > 0.0f ? sp_axis.ibus * sp_axis.vbus : 0.0f);
		stream_write(rt_report_msg);
	}

	// memcpy(&last_report,&report,sizeof(report));
}

void sp_ms_event(){
	uint32_t ms = hal.get_elapsed_ticks();
	static uint32_t last_1ms = 0, last_50ms = 0, last_250ms = 0;

	if (ms >= last_1ms + 1){
		last_1ms = ms;
	
  
	}
	else if (ms >= last_50ms + 50){
		last_50ms = ms;
		// if(diff.state != DIFF_IDLE && sp_axis.isAlive) {
		// 	// if (flag.got_rpm_diff){
		// 	sprintf(msg_debug,"Override state=%u start=%u act=%u diff=%.f atS=%u cur=%.1f", 
		// 		(uint8_t)diff.state,diff.f_start,sys.override.feed_rate,rpm_diff,sp_state.at_speed,sp_axis.lim_current);
		// 	report_message(msg_debug,Message_Plain);
		// 	memset(&msg_debug,0,sizeof(msg_debug));
		// 	// }
		// }
	}
	else if (ms >= last_250ms + 250){
		last_250ms = ms;
		// char c_msg[70];
		// sprintf(c_msg,"SP_State on=%u RPM: soll=%.0f ist=%.0f atS=%d waitACC=%d",
		// 	sp_state.on, sp_data.rpm_programmed, sp_data.rpm, sp_state.at_speed, flag.wait_accel);
		// report_message(c_msg,Message_Plain);
	}
}

void sp_execute_realtime(uint_fast16_t state){
	static uint32_t last_ms;
	on_execute_realtime(state);
    UNUSED(state);
    uint32_t ms = hal.get_elapsed_ticks();
		
	if (ms == last_ms)
		return;
	
	last_ms = ms;
	
	if (msg_debug[0]){
		// const char *_msg = msg_debug;
		report_message(msg_debug,Message_Plain);
		msg_debug[0] = 0;
	}

	if (diff.state != DIFF_IDLE)
		spindle_rpm_diff(state);

	if (sp_axis.isAlive /*&& hal.stream.connected*/){

		for(uint8_t idx = 0; idx < (sizeof(frames_periodic) / sizeof(periodic_frame_t)); idx++){
			if(frames_periodic[idx].active && (ms >= (frames_periodic[idx].last + frames_periodic[idx].interval))) {
				odrive_get_parameter(frames_periodic[idx].cmd_id,false);
				frames_periodic[idx].last = ms;
				// break;
			}
		}

		handle_cooling();

		if ((ms - sp_axis.last_heartbeat) > odrive.can_timeout){
			sp_axis.isAlive = false;
			memset(&sp_axis,0,sizeof(sp_axis));
			memset(&flag,0,sizeof(flag));
			hal.spindle.get_data = NULL;
			report_message("CAN timeout",Message_Info);
		}
		
	}
	
	// sp_ms_event();

	// if (sp_axis.isAlive){
	// 	if (flag.got_rpm_diff){
	// 		spindle_rpm_diff();
	// 		// if (sys.suspend)
	// 			// protocol_execute_rt();
	// 			// protocol_execute_realtime();
	// 			// protocol_enqueue_rt_command(on_execute_realtime);
	// 	}
	//
	// 	if ((ms - sp_axis.last_heartbeat) > odrive.can_timeout){
	// 		sp_axis.isAlive = false;
	// 		memset(&sp_axis,0,sizeof(sp_axis));
	// 		for(uint_fast8_t idx = 0; idx < (sizeof(frames_periodic) / sizeof(periodic_frame_t)); idx++)
	// 			frames_periodic[idx].active = Off;
	// 		hal.spindle.get_data = NULL;
	// 		report_message("CAN timeout",Message_Info);
	// 	}
	// 	periodic_frame_t *sent_msg = next_frame;
	// 	next_frame = sent_msg->next;
	// 	if(sent_msg->active && (ms > (sent_msg->last + sent_msg->interval))) {
	// 		sent_msg->last = ms;
	// 		odrive_get_parameter(sent_msg->cmd_id,false);
	// 	}
	//
	// 	// static uint32_t print_ticks = 250;
	//     // if(!(--print_ticks) || gotDiff) {
    // 	//     // sprintf(msg_debug,"Spindel Angel:%3.2f",sp_data.angular_position);
	// 	// 	// report_feedback;
	// 	// 	// float tol = settings.spindle.at_speed_tolerance;
	// 	// 	int diff_max = sp_data.rpm_programmed * rpm_tol;
	// 	// 	sprintf(msg_debug,"RPM diff=%3.1f max=%d tol=%.2f on=%u soll=%.0f ist=%.0f atS=%d waitS=%d flag=%d",
	// 	// 		rpm_diff, diff_max, rpm_tol, sp_state.on, sp_data.rpm_programmed, sp_data.rpm, sp_state.at_speed, flag.wait_accel, gotDiff);
	// 	// 	report_feedback;
    //     // 	print_ticks = gotDiff ? 10 : 250;
    // 	// }
	// }
	
}

//----------------------------Init and Setup-----------------------------------------

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:ODRIVE CAN v0.01]" ASCII_EOL);
    }
}

static void odrive_warning (uint_fast16_t state)
{
	if (msg_warning[0] != 0){
		report_message(msg_warning, Message_Plain);
		memset(&msg_warning,0,sizeof(msg_warning));
	}
	if(nvs_address == 0 && msg_warning[0] == 0)
    	report_message("ODrive failed to initialize! nvs_address=0", Message_Warning);
}

static void report_rpm_override_stats (sys_state_t state)
{	
	char msg_out[40];
    sprintf(msg_out,"Spindel Angel=%3.2f Pos=%04u",sp_data.angular_position, (uint16_t)(sp_axis.count_cpr));
	hal.stream.write(msg_out);
	hal.stream.write(ASCII_EOL);
    // format output -> T:21.17 /0.0000 B:21.04 /0.0000 @:0 B@:0
}

static void odrive_on_program_completed (program_flow_t program_flow, bool check_mode)
{
    // bool completed = program_flow == ProgramFlow_Paused; // || program_flow == ProgramFlow_CompletedM30;

    // if(frewind) {
    //     f_lseek(file.handle, 0);
    //     file.pos = file.line = 0;
    //     file.eol = false;
    //     hal.stream.read = await_cycle_start;
    //     if(grbl.on_state_change != trap_state_change_request) {
    //         state_change_requested = grbl.on_state_change;
    //         grbl.on_state_change = trap_state_change_request;
    //     }
    //     protocol_enqueue_rt_command(sdcard_restart_msg);
    // } else
    //     sdcard_end_job();

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static user_mcode_t userMCodeCheck (user_mcode_t mcode)
{
    return (uint32_t)mcode == Odrive_Set_Current_Limit || (uint32_t)mcode == Odrive_Reboot || 
				(uint32_t)mcode == Odrive_Report_Stats || (uint32_t)mcode == Odrive_Clear_Errors ||
            	(uint32_t)mcode == Odrive_Request_State || (uint32_t)mcode == Odrive_Set_Fan
			? mcode
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block, parameter_words_t *value_words)
{
    status_code_t state = Status_GcodeUnsupportedCommand;

    switch((uint32_t)gc_block->user_mcode) {

        case Odrive_Set_Current_Limit:{
			state = Status_OK;
        	parameter_words_t  word = *value_words;
         	if(word.f && isnan(gc_block->values.f))
           		state = Status_BadNumberFormat;
			else if (gc_block->values.f < 10.0f || gc_block->values.f > 60.0f)
				state = Status_GcodeValueOutOfRange;
			else
            	(*value_words).f = Off;
            break;}

        case Odrive_Request_State:{
			state = Status_OK;
         	if((*value_words).s && isnan(gc_block->values.s))
             	state = Status_BadNumberFormat;
			else if (gc_block->values.s < AXIS_STATE_IDLE || gc_block->values.s > AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION)
				state = Status_GcodeValueOutOfRange;
			else {
				(*value_words).s = Off;
           		// gc_block->user_mcode_sync = true;
			}            
            break;}

        case Odrive_Set_Fan:{
			state = Status_OK;
         	if((*value_words).s && isnan(gc_block->values.s))
             	state = Status_BadNumberFormat;
			else if((*value_words).f && isnan(gc_block->values.f))
				state = Status_BadNumberFormat;
			// else if (isnan(gc_block->values.s) || isnan(gc_block->values.f))
			// 	state = Status_GcodeValueOutOfRange;
			else {
				(*value_words).s = Off;
				(*value_words).f = Off;
           		// gc_block->user_mcode_sync = true;
			}            
            break;}
		case Odrive_Reboot:
		case Odrive_Clear_Errors:
        case Odrive_Report_Stats:
            state = Status_OK;
            break;

        default:{
            state = Status_Unhandled;
            break;}
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, value_words) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch((uint32_t)gc_block->user_mcode) {

        case Odrive_Set_Current_Limit:{
            odrive_set_limits(odrive.controller_vel_limit,gc_block->values.f, true);
			sprintf(msg_warning,"ODrive updating motor current to %02.1f",gc_block->values.f);
			protocol_enqueue_rt_command(odrive_warning);
            break;}

        case Odrive_Reboot:{ // Request rpm override stats report
            protocol_enqueue_rt_command(report_rpm_override_stats);
            break;}

        case Odrive_Clear_Errors:{
            hal.stream.write("ODrive clear errors ");
            if (odrive_clear_errors(true))
            	hal.stream.write("OK");
			else
				hal.stream.write("FAIL");
            break;}

        case Odrive_Request_State:{
			char msg[30];
            sprintf(msg,"ODrive request state %u ",(uint8_t)gc_block->values.s);
			hal.stream.write(msg);
            if (odrive_set_state((uint8_t)gc_block->values.s,true))
            	hal.stream.write("OK" ASCII_EOL);
			else
				hal.stream.write("FAIL" ASCII_EOL);
            break;}
			
		case Odrive_Set_Fan:{
			char msg[30];
            sprintf(msg,"ODrive set Fan %u val %u ",(uint8_t)gc_block->values.f, (uint16_t)gc_block->values.s);
			hal.stream.write(msg);
			pca9685_pwm_set_duty((uint8_t)gc_block->values.f,(uint16_t)gc_block->values.s);
            break;}

        case Odrive_Report_Stats:{
			CAN_message_t msg = {0};
			msg.id = set_msg_id(MSG_GET_ENCODER_COUNT,odrive.can_node_id);
			msg.len = 8;
			msg.flags.remote = On;
			s_msg = canbus_write_sync_msg(&msg,true);
			// sprintf(msg_feedback,"adding sync msg = %u",msg.mb);
			// report_feedback;
			while (!(s_msg->ready));
			sprintf(msg_feedback,"sync:%u t=%lu low:%lu high:%lu",s_msg->ready,s_msg->time_end-s_msg->time,msg.low,msg.high);
			report_feedback;
			canbus_write_sync_msg(&msg,false);
			gc_block->user_mcode_sync = true;
            break;}

        case 115:{
            hal.stream.write("FIRMWARE_NAME:grblHAL ");
            hal.stream.write("FIRMWARE_URL:https%3A//github.com/grblHAL ");
            hal.stream.write("FIRMWARE_VERSION:" GRBL_VERSION " ");
            hal.stream.write("FIRMWARE_BUILD:" GRBL_VERSION_BUILD ASCII_EOL);
            break;}

        default:{
            handled = false;
            break;}
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void odrive_settings_changed (settings_t *settings)
{

    if(init_ok && settings_changed)
        settings_changed(settings);
	else {
		odrive_settings_load();
        hal.spindle.set_state = spindleSetState;
    	hal.spindle.get_state = spindleGetState;
    	hal.spindle.update_rpm = spindleUpdateRPM;
		hal.spindle.get_data = *spindleGetData;
    	hal.spindle.reset_data = spindleDataReset;
		hal.driver_cap.variable_spindle = On;
    	hal.driver_cap.spindle_at_speed = On;
    	hal.driver_cap.spindle_dir = On;
		// if(spindle_encoder.ppr != settings->spindle.ppr) {
        // hal.spindle.reset_data = spindleDataReset;
        // hal.spindle.set_state((spindle_state_t){0}, 0.0f);
        // pidf_init(&spindle_tracker.pid, &settings->position.pid);
        // float timer_resolution = 1.0f / 1000000.0f; // 1 us resolution
        // spindle_tracker.min_cycles_per_tick = (int32_t)ceilf(settings->steppers.pulse_microseconds * 2.0f + settings->steppers.pulse_delay_microseconds);
        // spindle_encoder.ppr = settings->spindle.ppr;
        // spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
        // spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
        // spindle_encoder.maximum_tt = (uint32_t)(2.0f / timer_resolution) / spindle_encoder.tics_per_irq;
        // spindle_encoder.rpm_factor = 60.0f / ((timer_resolution * (float)spindle_encoder.ppr));
        // spindleDataReset();
        // }
		canbus_begin(0,1000*odrive.can_baudrate);
		pca9685_init();
	}

    init_ok = true;
}

static void reset(){

	// init_ok = false;
	// sp_axis = (axis_parameters_t){0};
	// sp_data = (spindle_data_t){0};
	// sp_state = (spindle_state_t){0};
	// flag = (flag_t){0};
	// diff = (rpm_diff_t){0};
	fan.state = FAN_STOP;
    driver_reset();
}

void odrive_init()
{
	if ((nvs_address = nvs_alloc(sizeof(odrive_settings_t)))) {

		memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

    	hal.user_mcode.check = userMCodeCheck;
    	hal.user_mcode.validate = userMCodeValidate;
    	hal.user_mcode.execute = userMCodeExecute;

    	driver_reset = hal.driver_reset;
        hal.driver_reset = reset;

    	settings_changed = hal.settings_changed;
    	hal.settings_changed = odrive_settings_changed;

    	on_report_options = grbl.on_report_options;
    	grbl.on_report_options = onReportOptions;

		on_execute_realtime = grbl.on_execute_realtime;
    	grbl.on_execute_realtime = sp_execute_realtime;

		on_unknown_feedback_message = grbl.on_unknown_feedback_message;
		grbl.on_unknown_feedback_message = sp_feedback_message;

		on_realtime_report = grbl.on_realtime_report;
		grbl.on_realtime_report = sp_rt_report;
	
		details.on_get_settings = grbl.on_get_settings;
    	grbl.on_get_settings = onReportSettings;

		on_program_completed = grbl.on_program_completed;
        grbl.on_program_completed = odrive_on_program_completed;

		odrive_settings_changed(&settings);

		// next_frame = &frames_periodic[0];
		// canbus_init();

    	for(uint8_t idx = 0; idx < (sizeof(frames_periodic) / sizeof(periodic_frame_t)); idx++){
			frames_periodic[idx].last = 0UL;
			// if (frames_periodic[idx].cmd_id == MSG_GET_ENCODER_ESTIMATES)
				// encoder_estimates_frame = &frames_periodic[idx];
		}

		odrv_listener.generalCallbackActive = On;
		odrv_listener.callbacksActive = 0ULL;
    	odrv_listener.frameHandler = (void *)&cb_gotFrame;
    	canbus_attachObj(&odrv_listener);

	}

	if(nvs_address == 0)
        protocol_enqueue_rt_command(odrive_warning);

    pinMode(LED_BUILTIN,OUTPUT);
	// return nvs_address != 0;
}

#endif