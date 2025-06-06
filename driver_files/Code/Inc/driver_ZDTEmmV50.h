/*
 * driver_ZDTEmmV50.h
 *
 *  Created on: May 29, 2025
 *      Author: LeoG // TODO: ZDT CAN Setup (with filter def)
 */

#ifndef INC_DRIVER_ZDTEMMV50_H_
#define INC_DRIVER_ZDTEMMV50_H_

#include <string.h>
#include "driver_can.h"

/* === Main structure definitions === */
typedef enum /* Return codes from ZDT related commands */
{
  ZDT_OK                 = 0x00,
  ZDT_ERROR              = 0x01,
  ZDT_CONDITIONS_NOT_MET = 0x02,
  ZDT_TIMEOUT 			 = 0x03,
} ZDT_ReturnCode;

typedef enum /* Checksum types */
{
	ZDT_CS0x6B   = 0x00,
	ZDT_CSXOR    = 0x01,
	ZDT_CSCRC8   = 0x02,
	ZDT_CSModbus = 0x03,
} ZDT_Checksum;

typedef struct { /* ZDT handler object */
	FDCAN_HandleTypeDef *hfdcan; // FDCAN_HandleTypeDef structure handling the CAN connection
	uint8_t mot_id;             // ID of the motor
	ZDT_Checksum checksum;       // Checksum to use when communicating
	int send_timeout;           // Timeout used when sending commands
	int receive_timeout;        // Timeout used when receiving commands
} ZDT_Handler;

/* === Argument structures === */
typedef enum /* Turning directions (speed or position) */
{
	ZDT_CW = 0x00, // CLockwise
	ZDT_CCW = 0x01, // Counter-clockwise
} ZDT_Dir;

typedef enum /* Homing mode types */
{
	ZDT_HomNear      = 0x00, // Homing mode: Nearest, Go to homing 0-position the nearest way (can go clockwise or counterclockwise). Will rotate 1 turn max.
	ZDT_HomDirNear   = 0x01, // Homing mode: Directional Nearest, Rotates in given direction up to reaching 0-position (nearest but one way). Will rotate 1 turn max.
	ZDT_HomCol       = 0x02, // Homing mode: Collision, The motor will rotate until it detects a collision (see criteria below). Can rotate without limits.
	ZDT_HomLimSwitch = 0x03, // Homing mode: Limit Switch, The motor will rotate until the external limit switch is triggered. Please refer to ZDT's documentation for the wiring. Can rotate without limits.
} ZDT_HomMode;

typedef struct { /* Homing mode parameters, used in 0x22 (read) and 0x4C (modify) */
	ZDT_HomMode mode; // ZDT_HomNear, ZDT_HomDirNear, ZDT_HomCol, ZDT_HomLimSwitch
	ZDT_Dir direction;             // ZDT_CW, ZDT_CCW
	uint16_t speed;       // Speed in RPM
	uint32_t timeout;    // Timeout in ms.
	uint16_t detection_speed; // Limitless Collision Homing Detection Speed in RPM.
	uint16_t detection_current; // Limitless Collision Homing Detection Current in mA.
	uint16_t detection_time; // Limitless Collision Homing Detection Time in ms.
	bool enable_pwr_on_auto_trigger; // Enable Power-On Auto-Trigger Homing Function. False is not enabled, True is enabled.
} ZDT_HomingParams;

typedef struct { /* Motor Status Flags, used in 0x3A (read) */
	bool enabled; // If Motor Enable Status Flag is True.
	bool in_pos; // If Motor In-Position Flag is True.
	bool stalled; // If Motor Stall Flag is True.
	bool stall_protection; // If Motor Stall Protection Flag is True.
} ZDT_MotorStatusFlags;

typedef struct { /* Homing Status Flags, used in 0x3B (read) */
    bool enc_ready; // If Encoder Ready Status Flag is True.
    bool CAL_table_ready; // If Calibration Table Ready Status Flag is True.
    bool homing_in_progress; // If Homing in Progress Flag is True.
    bool homing_failure; // If Homing Failure Flag is True."""
} ZDT_HomingStatusFlags;

typedef struct { /* Drive Configuration Parameters, used in 0x42 (read) and 0x48 (modify) commands */
	uint8_t received_bytes; // Number of bytes in the command
	uint8_t num_config_params; // Number of configuration parameters in the command
	uint8_t mot_type; // Stepmotor type. Examples; // 50 is 0.9°, 25 is 1.8°
	uint8_t pulse_port_control_mode; // 0 is PUL_OFF, 1 is PUL_OPEN, 2 is PUL_FOC, 3 is ESI_RCO.
	uint8_t communication_port_multiplexing_mode; // 0 is RxTx_OFF, 1 is ESI_ALO, 2 is UART_FUN, 3 is CAN1_MAP.
	uint8_t en_pin_active_level; // 0 is L, 1 is H, 2 is Hold (Always active)
	uint8_t dir_pin_effective_direction; // 0 is Clockwise, 1 is Counter-clockwise
	uint8_t microstep; // 0 is 256 microsteps, else number of microsteps
	bool microstep_interpolation; // True is Enabled, False is Disabled
	bool auto_screen_off; // True is Enabled, False is Disabled
	uint16_t open_loop_mode_operating_current; // in mA
	uint16_t closed_loop_mode_maximum_stall_current; // in mA
	uint16_t closed_loop_mode_maximum_output_voltage; // in mA
	uint8_t serial_baud_rate; // screen option order TODO; // 0x00; // 9600, 0x05; // 115200
	uint8_t CAN_communication_rate; // screen option order TODO; // 0x00; // 10000, 0x07; // 500000
	uint8_t ID_address; // direct info, for Serial/RS232/RS485/CAN common
	uint8_t communication_checksum_method; // 0; // 0x6B, 1; // XOR, 2; // CRC-8, 3; // Modbus
	uint8_t control_command_response; // TODO; // find in menu and match. 1; //  Control command list only returns confirmation of received command
	bool stall_protection; // False is Disabled, True is Enabled
	uint16_t stall_protection_speed_threshold; // in RPM
	uint16_t stall_protection_current_threshold; // in mA
	uint16_t stall_protection_detection_time_threshold; // in ms
	uint16_t position_arrival_window; // in 0.1° (0xA being 1°)
} ZDT_DriveConfigParams;

typedef struct { /* System Status Parameters, used in 0x43 (read) command. */
	uint8_t received_bytes; // Number of bytes in the command
	uint8_t num_config_params; // Number of configuration parameters in the command
	uint16_t bus_voltage; // In mV.
	uint16_t bus_phase_current; // In mA.
	uint16_t calibrated_encoder_value; // 0-65535 being a full rotation.
	ZDT_Dir dir_motor_target_position; // Direction for motor_target_position
	uint32_t motor_target_position; // 0-65535 being a full rotation.
	ZDT_Dir dir_motor_real_time_speed; // Direction for motor_real_time_speed
	uint16_t motor_real_time_speed; // In RPM.
	ZDT_Dir dir_motor_real_time_position; // Direction for motor_real_time_position
	uint32_t motor_real_time_position; // 0-65535 being a full rotation.
	ZDT_Dir dir_motor_position_error; // Direction for motor_position_error
	uint32_t motor_position_error; // 0-32767=0x7FFF being a full turn.
	bool flag_encoder_ready_status; //Whether encoder ready.
	bool flag_calibration_ready_status; //Whether calibration ready
	bool flag_homing_in_progress; //Whether Homing in progress.
	bool flag_homing_failure; //Whether Homing failure.!
	bool flag_enable_motor_status; //Whether motor enabled.
	bool flag_motor_in_position_flag; //Whether motor is in position.
	bool flag_motor_stall; //Whether motor is in stall mode.
	bool flag_stall_protection; //Whether motor is stall protected.
} ZDT_SystemStatusParams;

/* === Misc function prototypes === */
void *cpy_le(void *dest, const void *src, size_t type_size); // memcpy with byte swapping (Little-endian system vs big-endian CAN data)
ZDT_ReturnCode ZDT_CAN_setup(ZDT_Handler *zdthdl); // Init ZDT object and related CAN config. Should be used for each ZDT_Handler used.
ZDT_ReturnCode ZDT_CAN_send(ZDT_Handler *zdthdl, uint32_t can_id, uint8_t *TxData, uint8_t size); // Wrapper for CAN_send made for ZDT handler
ZDT_ReturnCode ZDT_CAN_receive_from(ZDT_Handler *zdthdl, uint32_t can_id, uint8_t *RxData, uint8_t *size); // Wrapper for CAN_receive made for ZDT handler /* TODO Should also check for mot id, and loop like that ! */
ZDT_ReturnCode ZDT_CAN_clear_queues_of(uint8_t mot_id); // Clear all message queues of given mot_id

int _ZDT_compute_checksum(ZDT_Handler *zdthdl, uint8_t *data, uint8_t size, uint8_t *cs_value); // Compute checksum
bool _ZDT_check_checksum(ZDT_Handler *zdthdl, uint8_t *data, uint8_t total_size); // For packets received

/* === ZDT command prototypes === */
ZDT_ReturnCode ZDT_cmd_trigger_CAL(ZDT_Handler *zdthdl); // Command 0x06
ZDT_ReturnCode ZDT_cmd_reset_position_to_0(ZDT_Handler *zdthdl); // Command 0x0A
ZDT_ReturnCode ZDT_cmd_release_stall_protection(ZDT_Handler *zdthdl); // Command 0x0E
ZDT_ReturnCode ZDT_cmd_factory_reset(ZDT_Handler *zdthdl); // Command 0x0F
ZDT_ReturnCode ZDT_cmd_read_firmware_and_hardware_version(ZDT_Handler *zdthdl, uint8_t *firmware_version, uint8_t *hardware_version); // Command 0x1F
ZDT_ReturnCode ZDT_cmd_read_phase_resistance_and_inductance(ZDT_Handler *zdthdl, uint16_t *resistance, uint16_t *inductance); // Command 0x20
ZDT_ReturnCode ZDT_cmd_read_position_PID_parameters(ZDT_Handler *zdthdl, uint32_t *kp, uint32_t *ki, uint32_t *kd); // Command 0x21
ZDT_ReturnCode ZDT_cmd_read_homing_parameters(ZDT_Handler *zdthdl, ZDT_HomingParams *hom_params); // Command 0x22
ZDT_ReturnCode ZDT_cmd_read_bus_voltage(ZDT_Handler *zdthdl, uint16_t *voltage); // Command 0x24
ZDT_ReturnCode ZDT_cmd_read_phase_current(ZDT_Handler *zdthdl, uint16_t *current); // Command 0x27
ZDT_ReturnCode ZDT_cmd_read_CAL_encoder_value(ZDT_Handler *zdthdl, uint16_t *lin_enc_val); // Command 0x31
ZDT_ReturnCode ZDT_cmd_read_input_pulse_count(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *pulse_count); // Command 0x32
ZDT_ReturnCode ZDT_cmd_read_closed_loop_target_position(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *target_pos); // Command 0x33
ZDT_ReturnCode ZDT_cmd_read_open_loop_target_position(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *target_pos); // Command 0x34
ZDT_ReturnCode ZDT_cmd_read_current_speed(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint16_t *speed); // Command 0x35
ZDT_ReturnCode ZDT_cmd_read_current_position(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *position); // Command 0x36
ZDT_ReturnCode ZDT_cmd_read_current_position_error(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *position_error); // Command 0x37
ZDT_ReturnCode ZDT_cmd_read_motor_status_flags(ZDT_Handler *zdthdl, ZDT_MotorStatusFlags *mot_flags); // Command 0x3A
ZDT_ReturnCode ZDT_cmd_read_homing_status_flags(ZDT_Handler *zdthdl, ZDT_HomingStatusFlags *hom_flags); // Command 0x3B
ZDT_ReturnCode ZDT_cmd_read_drive_config_parameters(ZDT_Handler *zdthdl, ZDT_DriveConfigParams *drive_config_params); // Command 0x42
ZDT_ReturnCode ZDT_cmd_read_system_status_parameters(ZDT_Handler *zdthdl, ZDT_SystemStatusParams *system_status_params); // Command 0x43
ZDT_ReturnCode ZDT_cmd_edit_open_loop_operating_current(ZDT_Handler *zdthdl, bool do_store, uint16_t ol_mode_current); // Command 0x44
ZDT_ReturnCode ZDT_cmd_switch_open_loop_closed_loop(ZDT_Handler *zdthdl, bool do_store, bool new_as_open); // Command 0x46
ZDT_ReturnCode ZDT_cmd_edit_drive_config_parameters(ZDT_Handler *zdthdl, bool do_store, ZDT_DriveConfigParams drive_config_params); // Command 0x48
ZDT_ReturnCode ZDT_cmd_edit_position_PID_parameters(ZDT_Handler *zdthdl, bool do_store, uint32_t kp, uint32_t ki, uint32_t kd); // Command 0x4A
ZDT_ReturnCode ZDT_cmd_edit_homing_parameters(ZDT_Handler *zdthdl, bool do_store, ZDT_HomingParams hom_params); // Command 0x4C
ZDT_ReturnCode ZDT_cmd_allow_divide10_on_com_speed_cmd(ZDT_Handler *zdthdl, bool do_store, bool do_divide10); // Command 0x4F
ZDT_ReturnCode ZDT_cmd_edit_microstep(ZDT_Handler *zdthdl,  bool do_store, uint8_t microstep_value); // Command 0x84
ZDT_ReturnCode ZDT_cmd_set_0_position_homing(ZDT_Handler *zdthdl, bool do_store); // Command 0x93
ZDT_ReturnCode ZDT_cmd_trigger_homing(ZDT_Handler *zdthdl, ZDT_HomMode hom_mode, bool m_m_flag); // Command 0x9A
ZDT_ReturnCode ZDT_cmd_interrupt_homing(ZDT_Handler *zdthdl); // Command 0x9C
ZDT_ReturnCode ZDT_cmd_edit_ID_address(ZDT_Handler *zdthdl, bool do_store, uint8_t new_mot_id); // Command 0xAE
ZDT_ReturnCode ZDT_cmd_motor_enable_control(ZDT_Handler *zdthdl, bool do_enable, bool m_m_flag); // Command 0xF3
ZDT_ReturnCode ZDT_cmd_speed_mode_control(ZDT_Handler *zdthdl, ZDT_Dir dir, uint16_t speed, uint8_t accel_level, bool m_m_flag); // Command 0xF6
ZDT_ReturnCode ZDT_cmd_auto_run_on_power_on(ZDT_Handler *zdthdl, ZDT_Dir dir, uint8_t store_clear_flag, uint16_t speed, uint8_t accel_level, bool do_en_pin_control); // Command // TODO store_clear_flag?
ZDT_ReturnCode ZDT_cmd_position_mode_control(ZDT_Handler *zdthdl, ZDT_Dir dir, uint32_t pulse_count, uint16_t speed, uint8_t accel_level, bool is_relative, bool m_m_flag); // Command 0xFD
ZDT_ReturnCode ZDT_cmd_immediate_stop(ZDT_Handler *zdthdl, bool m_m_flag); // Command 0xFE
ZDT_ReturnCode ZDT_cmd_M_M_synchronized_motion(ZDT_Handler *zdthdl, bool do_broadcast); // Command 0xFF
ZDT_ReturnCode ZDT_receive_reached_message(ZDT_Handler *zdthdl, uint32_t timeout); // Wait for {0xFD, 0x9F, CS} Reached message

#endif /* INC_DRIVER_ZDTEMMV50_H_ */
