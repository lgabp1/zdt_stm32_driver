/*
 * driver_ZDTEmmV50.c
 *
 *  Created on: May 29, 2025
 *      Author: LeoG
 */
#include "driver_ZDTEmmV50.h"


/*
 * memcpy with byte swapping (Little-endian system vs big-endian CAN data)
 */
void *cpy_le(void *dest, const void *src, size_t type_size)
{
    memcpy(dest, src, type_size);

    // Swap bytes
    uint8_t *ptr = (uint8_t *)dest;
    for (size_t i = 0; i < type_size / 2; i++)
    {
        uint8_t temp = ptr[i];
        ptr[i] = ptr[type_size - 1 - i];
        ptr[type_size - 1 - i] = temp;
    }
    return dest;
}

/*
 * Compute corresponding checksum //TODO: implement other checksums, ajouter mot_id danas le calcul !
 */
int _ZDT_compute_checksum(ZDT_Handler *zdthdl, uint8_t *data, uint8_t size, uint8_t *cs_value)
{
	switch ((*zdthdl).checksum)
	{
		case ZDT_CS0x6B:
			*cs_value = 0x6BU;
			return 0;
		case ZDT_CSXOR:
			return 1;
		case ZDT_CSCRC8:
			return 1;
		case ZDT_CSModbus:
			return 1;
		default:
			return 1; // Invalid checksum
	}
}

/*
 * For packets received
 */
bool _ZDT_check_checksum(ZDT_Handler *zdthdl, uint8_t *data, uint8_t total_size)
{
	if (total_size < 2) {return false;} // Too tiny to both have checksum and data

	uint8_t cs_value;
	if(_ZDT_compute_checksum(zdthdl, data, total_size-1, &cs_value)){return false;} // Compute cs
	if (cs_value != data[total_size-1]) {return false;} // Test if cs compatible
	return true;
}


/*
 * Init ZDT object and related CAN config. Should be used for each ZDT_Handler used.
 */
ZDT_ReturnCode ZDT_CAN_setup(ZDT_Handler *zdthdl)
{
	/* CAN Init */
	uint32_t can_ids_to_track[5];
	for (int i=0; i<5; i++)
	{
		can_ids_to_track[i] = (((uint32_t)(zdthdl->mot_id)) <<8) + i; // Add the five sub can_ids corresponding to motor id
	}

	if(CAN_start((zdthdl->hfdcan), can_ids_to_track, 5)!= HAL_OK)
	{
	  Error_Handler();
	}
	return ZDT_OK;
}

/*
 * Wrapper for CAN_send made for ZDT handler
 */
ZDT_ReturnCode ZDT_CAN_send(ZDT_Handler *zdthdl, uint32_t can_id, uint8_t *TxData, uint8_t size)
{
	HAL_StatusTypeDef ret;
	ret = CAN_send( (zdthdl->hfdcan), can_id, true, TxData, size);
	if (ret != HAL_OK) {return ZDT_ERROR;}
	return ZDT_OK;
}

/*
 * Wrapper for CAN_receive made for ZDT handler with timeout
 */
ZDT_ReturnCode ZDT_CAN_receive_from_timeout(ZDT_Handler *zdthdl, uint32_t timeout, uint32_t can_id, uint8_t *RxData, uint8_t *size)
{
	HAL_StatusTypeDef ret; FDCAN_RxHeaderTypeDef RxHeaderBuff;

	ret = CAN_receive_from( (zdthdl->hfdcan), can_id, timeout, RxData, &RxHeaderBuff);
	if (ret != HAL_OK) {return ZDT_ERROR;}
	ret = _CAN_from_DLC_to_SIZE(RxHeaderBuff.DataLength, size);
	if (ret != HAL_OK) {return ZDT_ERROR;}

	return ZDT_OK;
}

/*
 * Wrapper for CAN_receive made for ZDT handler
 */
ZDT_ReturnCode ZDT_CAN_receive_from(ZDT_Handler *zdthdl, uint32_t can_id, uint8_t *RxData, uint8_t *size)
{
	return ZDT_CAN_receive_from_timeout(zdthdl, zdthdl->receive_timeout, can_id, RxData, size);
}

/*
 * Clear all message queues of given mot_id
 */
ZDT_ReturnCode ZDT_CAN_clear_queues_of(uint8_t mot_id)
{
	for (uint8_t offset=0; offset<5; offset++)
	{
		uint32_t can_id = (mot_id<<8) + offset;
		/* Retrieve corresponding queue */
		CAN_MessageQueue_t *queuep;
		for (int ind = 0; ind<CANRxQueuesCount; ind++)
		{
			if (can_id == CANRxQueues[ind].can_id)
			{ // If not whitelisted, will be thrown away
				queuep = &(CANRxQueues[ind]);
				if (CAN_queue_clear(queuep)) {return ZDT_ERROR;}
				break;
			}
		}
	}
	return ZDT_OK;
}

/*
 * Command 0x06: Trigger Encoder Calibration
 * Trigger Encoder Calibration.
 */
ZDT_ReturnCode ZDT_cmd_trigger_CAL(ZDT_Handler *zdthdl)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x06; payload[1] = 0x45; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x06 && RxBuff[1] == 0xE2) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0x06 && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3)) {
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x0A: Reset Current Position Angle to 0.
 * Reset the current position angle, position error, and pulse count to zero.
 */
ZDT_ReturnCode ZDT_cmd_reset_position_to_0(ZDT_Handler *zdthdl)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x0A; payload[1] = 0x6D; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x0A && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x0E: Release Stall Protection
 * If a motor stall, send this command to release the stall protection. If motor is not in stalled mode, the conditions are not met.
 */
ZDT_ReturnCode ZDT_cmd_release_stall_protection(ZDT_Handler *zdthdl)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x0E; payload[1] = 0x52; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x0E && RxBuff[1] == 0xE2) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0x0E && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x0F: Factory Reset
 * Restore Factory Settings.
 */
ZDT_ReturnCode ZDT_cmd_factory_reset(ZDT_Handler *zdthdl)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x0F; payload[1] = 0x5F; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x0F && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x1F: Read Firmware Version and Corresponding Hardware Version.
 * Read Firmware Version and Corresponding Hardware Version.
 */
ZDT_ReturnCode ZDT_cmd_read_firmware_and_hardware_version(ZDT_Handler *zdthdl, uint8_t *firmware_version, uint8_t *hardware_version)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x1F; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 4) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x1F && _ZDT_check_checksum(zdthdl, RxBuff, 4))
	{
		*firmware_version = RxBuff[1];
		*hardware_version = RxBuff[2];
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x20: Read Phase Resistance and Inductance values
 * Read Phase Resistance and Phase Inductance values.
 */
ZDT_ReturnCode ZDT_cmd_read_phase_resistance_and_inductance(ZDT_Handler *zdthdl, uint16_t *resistance, uint16_t *inductance)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x20; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 6) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x20 && _ZDT_check_checksum(zdthdl, RxBuff, 6))
	{
		cpy_le(resistance, &RxBuff[1], 2); // copy 2 bytes (from BE)
		cpy_le(inductance, &RxBuff[3], 2); // copy 2 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x21: Read Position Loop PID Parameters
 * Read Position Loop PID Parameters.
 */
ZDT_ReturnCode ZDT_cmd_read_position_PID_parameters(ZDT_Handler *zdthdl, uint32_t *kpp, uint32_t *kip, uint32_t *kdp) // Command 0x21
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t RxDataBuff[14];
	payload[0] = 0x21; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize); // First message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x21)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[0], &RxBuff[0], 8); // Fill full data buffer, include function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, RxBuff, &RxSize); // Second message
	if ((ret) || (RxSize < 7) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x21)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[8], &RxBuff[1], 6); // Fill full data buffer, exclude function code

	if (_ZDT_check_checksum(zdthdl, RxDataBuff, 14))
	{
		cpy_le(kpp, &RxDataBuff[1], 4); // copy 4 bytes (from BE)
		cpy_le(kip, &RxDataBuff[5], 4); // copy 4 bytes (from BE)
		cpy_le(kdp, &RxDataBuff[9], 4); // copy 4 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x22: Read Origin Homing Parameters
 * Read Origin Homing Parameters.
 */
ZDT_ReturnCode ZDT_cmd_read_homing_parameters(ZDT_Handler *zdthdl, ZDT_HomingParams *hom_params)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t RxDataBuff[17];
	payload[0] = 0x22; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize); // First message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x22)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[0], &RxBuff[0], 8); // Fill full data buffer, include function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, RxBuff, &RxSize); // Second message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x22)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[8], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 2, RxBuff, &RxSize); // Third message
	if ((ret) || (RxSize < 3) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x22)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[15], &RxBuff[1], 2); // Fill full data buffer, exclude function code

	if (_ZDT_check_checksum(zdthdl, RxDataBuff, 17))
	{
		(*hom_params).mode = (ZDT_HomMode) RxDataBuff[1]; // Mode
		(*hom_params).direction = (ZDT_Dir) RxDataBuff[2]; // Direction
		cpy_le( &((*hom_params).speed), &RxDataBuff[3], 2); // copy 2 bytes (from BE)
		cpy_le( &((*hom_params).timeout), &RxDataBuff[5], 4); // copy 4 bytes (from BE)
		cpy_le( &((*hom_params).detection_speed), &RxDataBuff[9], 2); // copy 2 bytes (from BE)
		cpy_le( &((*hom_params).detection_current), &RxDataBuff[11], 2); // copy 2 bytes (from BE)
		cpy_le( &((*hom_params).detection_time), &RxDataBuff[13], 2); // copy 2 bytes (from BE)
		(*hom_params).enable_pwr_on_auto_trigger = (bool) RxDataBuff[15]; // Power on auto trigger
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x24: Read Bus Voltage
 * Read Bus Voltage.
 */
ZDT_ReturnCode ZDT_cmd_read_bus_voltage(ZDT_Handler *zdthdl, uint16_t *voltage)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x24; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 4) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x24 && _ZDT_check_checksum(zdthdl, RxBuff, 4))
	{
		cpy_le(voltage, &RxBuff[1], 2); // copy 2 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x27: Read Phase Current
 * Read Phase Current.
 */
ZDT_ReturnCode ZDT_cmd_read_phase_current(ZDT_Handler *zdthdl, uint16_t *current)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x27; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 4) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x27 && _ZDT_check_checksum(zdthdl, RxBuff, 4))
	{
		cpy_le(current, &RxBuff[1], 2); // copy 2 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x31: Read CAL Encoder values
 * Read Linearized Calibrated Encoder Value.
 */
ZDT_ReturnCode ZDT_cmd_read_CAL_encoder_value(ZDT_Handler *zdthdl, uint16_t *lin_enc_val)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x31; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 4) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x31 && _ZDT_check_checksum(zdthdl, RxBuff, 4))
	{
		cpy_le(lin_enc_val, &RxBuff[1], 2); // copy 2 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x32: Read Input Pulse Count
 * Read Input Pulse Count.
 */
ZDT_ReturnCode ZDT_cmd_read_input_pulse_count(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *pulse_count)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x32; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 6) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x32 && _ZDT_check_checksum(zdthdl, RxBuff, 6))
	{
		*dir = (ZDT_Dir) RxBuff[1]; // Direction
		cpy_le(pulse_count, &RxBuff[2], 4); // copy 4 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x33: Read Closed Loop Target Position.
 * Read Motor Target Position.
 */
ZDT_ReturnCode ZDT_cmd_read_closed_loop_target_position(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *target_pos)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x33; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 7) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x33 && _ZDT_check_checksum(zdthdl, RxBuff, 7))
	{
		*dir = (ZDT_Dir) RxBuff[1]; // Direction
		cpy_le(target_pos, &RxBuff[2], 4); // copy 4 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x34: Read Open Loop Target Pos
 * Read Motor Real-Time Set Target Position (Open-Loop Mode Real-Time Position).
 */
ZDT_ReturnCode ZDT_cmd_read_open_loop_target_position(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *target_pos)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x34; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 7) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x34 && _ZDT_check_checksum(zdthdl, RxBuff, 7))
	{
		*dir = (ZDT_Dir) RxBuff[1]; // Direction
		cpy_le(target_pos, &RxBuff[2], 4); // copy 4 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x35: Read Motor Real-Time Speed
 * Read Current Open-Loop Speed.
 */
ZDT_ReturnCode ZDT_cmd_read_current_speed(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint16_t *speed)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x35; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 5) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x35 && _ZDT_check_checksum(zdthdl, RxBuff, 5))
	{
		*dir = (ZDT_Dir) RxBuff[1]; // Direction
		cpy_le(speed, &RxBuff[2], 2); // copy 2 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x36: Read Motor Open-Loop current Position
 * Read Motor Real-Time Position.
 */
ZDT_ReturnCode ZDT_cmd_read_current_position(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *position)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x36; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 7) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x36 && _ZDT_check_checksum(zdthdl, RxBuff, 7))
	{
		*dir = (ZDT_Dir) RxBuff[1]; // Direction
		cpy_le(position, &RxBuff[2], 4); // copy 4 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x37: Read Current Motor Position Error
 * Read Current Motor Position Error.
 */
ZDT_ReturnCode ZDT_cmd_read_current_position_error(ZDT_Handler *zdthdl, ZDT_Dir *dir, uint32_t *position_error)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x37; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 7) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x37 && _ZDT_check_checksum(zdthdl, RxBuff, 7))
	{
		*dir = (ZDT_Dir) RxBuff[1]; // Direction
		cpy_le(position_error, &RxBuff[2], 4); // copy 4 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x3A: Read Motor Status Flag
 * Read Motor Status Flag.
 */
ZDT_ReturnCode ZDT_cmd_read_motor_status_flags(ZDT_Handler *zdthdl, ZDT_MotorStatusFlags *mot_flags)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x3A; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x3A && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		uint8_t flags_byte = RxBuff[1]; // Byte to mask
		mot_flags->enabled = (bool) ( (flags_byte & 0x01U) != 0 );
		mot_flags->in_pos = (bool) ( (flags_byte & 0x02U) != 0 );
		mot_flags->stalled = (bool) ( (flags_byte & 0x04U) != 0 );
		mot_flags->stall_protection = (bool) ( (flags_byte & 0x08U) != 0 );
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x3B: Read Homing Status Flag
 * Read Homing Status Flag.
 */
ZDT_ReturnCode ZDT_cmd_read_homing_status_flags(ZDT_Handler *zdthdl, ZDT_HomingStatusFlags *hom_flags)
{
	uint8_t payload[2]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x3B; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 1, &payload[1])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x3B && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		uint8_t flags_byte = RxBuff[1]; // Byte to mask
		hom_flags->enc_ready = (bool) ( (flags_byte & 0x01U) != 0 );
		hom_flags->CAL_table_ready = (bool) ( (flags_byte & 0x02U) != 0 );
		hom_flags->homing_in_progress = (bool) ( (flags_byte & 0x04U) != 0 );
		hom_flags->homing_failure = (bool) ( (flags_byte & 0x08U) != 0 );
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x42: Read Drive Configuration Parameters
 * Read Drive Configuration Parameters.
 */
ZDT_ReturnCode ZDT_cmd_read_drive_config_parameters(ZDT_Handler *zdthdl, ZDT_DriveConfigParams *drive_config_params)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t RxDataBuff[32];
	payload[0] = 0x42; payload[1] = 0x6C; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize); // First message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x42)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[0], &RxBuff[0], 8); // Fill full data buffer, include function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, RxBuff, &RxSize); // Second message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x42)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[8], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 2, RxBuff, &RxSize); // Third message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x42)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[15], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 3, RxBuff, &RxSize); // Fourth message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x42)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[22], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 4, RxBuff, &RxSize); // Fifth message
	if ((ret) || (RxSize < 4) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x42)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[29], &RxBuff[1], 3); // Fill full data buffer, exclude function code

	if (_ZDT_check_checksum(zdthdl, RxDataBuff, 32))
	{
		drive_config_params->received_bytes = RxDataBuff[1];
		drive_config_params->num_config_params = RxDataBuff[2];
		drive_config_params->mot_type = RxDataBuff[3];
		drive_config_params->pulse_port_control_mode = RxDataBuff[4];
		drive_config_params->communication_port_multiplexing_mode = RxDataBuff[5];
		drive_config_params->en_pin_active_level = RxDataBuff[6];
		drive_config_params->dir_pin_effective_direction = RxDataBuff[7];
		drive_config_params->microstep = RxDataBuff[8];
		drive_config_params->microstep_interpolation = (bool) (RxDataBuff[9] != 0);
		drive_config_params->auto_screen_off = (bool) (RxDataBuff[10] != 0);
		cpy_le( &(drive_config_params->open_loop_mode_operating_current), &RxDataBuff[11], 2); // copy 2 bytes (from BE)
		cpy_le( &(drive_config_params->closed_loop_mode_maximum_stall_current), &RxDataBuff[13], 2); // copy 2 bytes (from BE)
		cpy_le( &(drive_config_params->closed_loop_mode_maximum_output_voltage), &RxDataBuff[15], 2); // copy 2 bytes (from BE)
		drive_config_params->serial_baud_rate = RxDataBuff[17];
		drive_config_params->CAN_communication_rate = RxDataBuff[18];
		drive_config_params->ID_address = RxDataBuff[19];
		drive_config_params->communication_checksum_method = RxDataBuff[20];
		drive_config_params->control_command_response = RxDataBuff[21];
		drive_config_params->stall_protection = (bool) (RxDataBuff[22] != 0);
		cpy_le( &(drive_config_params->stall_protection_speed_threshold), &RxDataBuff[23], 2); // copy 2 bytes (from BE)
		cpy_le( &(drive_config_params->stall_protection_current_threshold), &RxDataBuff[25], 2); // copy 2 bytes (from BE)
		cpy_le( &(drive_config_params->stall_protection_detection_time_threshold), &RxDataBuff[27], 2); // copy 2 bytes (from BE)
		cpy_le( &(drive_config_params->position_arrival_window), &RxDataBuff[29], 2); // copy 2 bytes (from BE)
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}


/*
 * Command 0x43: Read System Status Parameters
 * Read System Status Parameters.
 */
ZDT_ReturnCode ZDT_cmd_read_system_status_parameters(ZDT_Handler *zdthdl, ZDT_SystemStatusParams *system_status_params)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t RxDataBuff[30];
	payload[0] = 0x43; payload[1] = 0x7A; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id))
	{
		return ZDT_ERROR;
	} // Clear message queues used by this device

	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	HAL_Delay(2000);

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize); // First message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x43)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[0], &RxBuff[0], 8); // Fill full data buffer, include function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, RxBuff, &RxSize); // Second message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x43)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[8], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 2, RxBuff, &RxSize); // Third message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x43)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[15], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 3, RxBuff, &RxSize); // Fourth message
	if ((ret) || (RxSize < 8) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x43)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[22], &RxBuff[1], 7); // Fill full data buffer, exclude function code
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 4, RxBuff, &RxSize); // Fifth message
	if ((ret) || (RxSize < 2) || ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) || (RxBuff[0] != 0x43)) {return ZDT_ERROR;} // Error or too tiny or Error message or wrong function code
	memcpy(&RxDataBuff[29], &RxBuff[1], 1); // Fill full data buffer, exclude function code

	if (_ZDT_check_checksum(zdthdl, RxDataBuff, 30))
	{
		system_status_params->received_bytes = RxDataBuff[1];
		system_status_params->num_config_params = RxDataBuff[2];
		cpy_le( &(system_status_params->bus_voltage), &RxDataBuff[3], 2); // copy 2 bytes (from BE)
		cpy_le( &(system_status_params->bus_phase_current), &RxDataBuff[5], 2); // copy 2 bytes (from BE)
		cpy_le( &(system_status_params->calibrated_encoder_value), &RxDataBuff[7], 2); // copy 2 bytes (from BE)
		system_status_params->dir_motor_target_position = (ZDT_Dir) ( RxDataBuff[9] );
		cpy_le( &(system_status_params->motor_target_position), &RxDataBuff[10], 4); // copy 4 bytes (from BE)
		system_status_params->dir_motor_real_time_speed = (ZDT_Dir) ( RxDataBuff[14] );
		cpy_le( &(system_status_params->motor_real_time_speed), &RxDataBuff[15], 2); // copy 2 bytes (from BE)
		system_status_params->dir_motor_real_time_position = (ZDT_Dir) ( RxDataBuff[17] );
		cpy_le( &(system_status_params->motor_real_time_position), &RxDataBuff[18], 4); // copy 4 bytes (from BE)
		system_status_params->dir_motor_position_error = (ZDT_Dir) ( RxDataBuff[22] );
		cpy_le( &(system_status_params->motor_position_error), &RxDataBuff[23], 4); // copy 4 bytes (from BE)
		uint8_t ready_status_flag_byte = RxDataBuff[28];
		system_status_params->flag_encoder_ready_status = (bool) ( (ready_status_flag_byte & 0x01U) != 0 );
		system_status_params->flag_calibration_ready_status = (bool) ( (ready_status_flag_byte & 0x02U) != 0 );
		system_status_params->flag_homing_in_progress = (bool) ( (ready_status_flag_byte & 0x04U) != 0 );
		system_status_params->flag_homing_failure = (bool) ( (ready_status_flag_byte & 0x08U) != 0 );
		uint8_t motor_status_flag_byte = RxDataBuff[29];
		system_status_params->flag_enable_motor_status = (bool) ( (motor_status_flag_byte & 0x01U) != 0 );
		system_status_params->flag_motor_in_position_flag = (bool) ( (motor_status_flag_byte & 0x02U) != 0 );
		system_status_params->flag_motor_stall = (bool) ( (motor_status_flag_byte & 0x04U) != 0 );
		system_status_params->flag_stall_protection = (bool) ( (motor_status_flag_byte & 0x08U) != 0 );

		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x44: Edit Open-Loop Operating Current
 * Modify Open-Loop Mode Operating Current
 */
ZDT_ReturnCode ZDT_cmd_edit_open_loop_operating_current(ZDT_Handler *zdthdl, bool do_store, uint16_t ol_mode_current)
{
	uint8_t payload[6]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x44; payload[1] = 0x33; // Fixed values
	payload[2] = (uint8_t) do_store;
	cpy_le( &(payload[3]), &ol_mode_current, 2); // copy 2 bytes (from LE)
	if (_ZDT_compute_checksum(zdthdl, payload, 5, &payload[5])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 6);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x44 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x46: Switch Open-Loop↔Closed-Loop modes
 * Switch Open-Loop/Closed-Loop Mode (P_Pul Menu Option).
 */
ZDT_ReturnCode ZDT_cmd_switch_open_loop_closed_loop(ZDT_Handler *zdthdl, bool do_store, bool new_as_open)
{
	uint8_t payload[5]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x46; payload[1] = 0x69; // Fixed values
	payload[2] = (uint8_t) do_store;
	if (new_as_open) {payload[3]=0x01;} else {payload[3]=0x02;}
	if (_ZDT_compute_checksum(zdthdl, payload, 4, &payload[4])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x46 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x48: Edit Drive Config Parameters
 * Modify Drive Configuration Parameters.
 */
ZDT_ReturnCode ZDT_cmd_edit_drive_config_parameters(ZDT_Handler *zdthdl, bool do_store, ZDT_DriveConfigParams drive_config_params)
{
	uint8_t payload[8]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t TxDataBuff[32];
	/* Filling full payload array */
	TxDataBuff[0] = 0x48; TxDataBuff[1] = 0xD1; // Fixed values
	TxDataBuff[2] = (uint8_t) do_store;
	TxDataBuff[3] = drive_config_params.mot_type;
	TxDataBuff[4] = drive_config_params.pulse_port_control_mode;
	TxDataBuff[5] = drive_config_params.communication_port_multiplexing_mode;
	TxDataBuff[6] = drive_config_params.en_pin_active_level;
	TxDataBuff[7] = drive_config_params.dir_pin_effective_direction;

	TxDataBuff[8] = drive_config_params.microstep;
	TxDataBuff[9] = (uint8_t) drive_config_params.microstep_interpolation;
	TxDataBuff[10] = (uint8_t) drive_config_params.auto_screen_off;
	cpy_le( &TxDataBuff[11], &(drive_config_params.open_loop_mode_operating_current), 2); // copy 2 bytes (from LE)
	cpy_le( &TxDataBuff[13], &(drive_config_params.closed_loop_mode_maximum_stall_current), 2); // copy 2 bytes (from LE)

	cpy_le( &TxDataBuff[15], &(drive_config_params.closed_loop_mode_maximum_output_voltage), 2); // copy 2 bytes (from LE)
	TxDataBuff[17] = drive_config_params.serial_baud_rate;
	TxDataBuff[18] = drive_config_params.CAN_communication_rate;
	TxDataBuff[19] = 0x01U; // Deprecated param
	TxDataBuff[20] = drive_config_params.communication_checksum_method;
	TxDataBuff[21] = drive_config_params.control_command_response;

	TxDataBuff[22] = (uint8_t) drive_config_params.stall_protection;
	cpy_le( &TxDataBuff[23], &(drive_config_params.stall_protection_speed_threshold), 2); // copy 2 bytes (from LE)
	cpy_le( &TxDataBuff[25], &(drive_config_params.stall_protection_current_threshold), 2); // copy 2 bytes (from LE)
	cpy_le( &TxDataBuff[27], &(drive_config_params.stall_protection_detection_time_threshold), 2); // copy 2 bytes (from LE)

	cpy_le( &TxDataBuff[29], &(drive_config_params.position_arrival_window), 2); // copy 2 bytes (from LE)
	if (_ZDT_compute_checksum(zdthdl, TxDataBuff, 31, &TxDataBuff[31])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	payload[0] = TxDataBuff[0];
	memcpy(&payload[1], &TxDataBuff[1], 7); // First payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[8], 7);// Second payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[15], 7);// Third payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 2, payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[22], 7);// Fourth payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 3, payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[29], 3);// Fifth payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 4, payload, 4);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x48 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x4A: Edit Pos Loop PID Parameters
 * Modify the position loop PID parameters.
 */
ZDT_ReturnCode ZDT_cmd_edit_position_PID_parameters(ZDT_Handler *zdthdl, bool do_store, uint32_t kp, uint32_t ki, uint32_t kd)
{
	uint8_t payload[8]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t TxDataBuff[16];
	/* Filling full payload array */
	TxDataBuff[0] = 0x4A; TxDataBuff[1] = 0xC3; // Fixed values
	TxDataBuff[2] = (uint8_t) do_store;
	cpy_le( &TxDataBuff[3], &kp, 4); // copy 4 bytes (from LE)
	cpy_le( &TxDataBuff[7], &ki, 4); // copy 4 bytes (from LE)
	cpy_le( &TxDataBuff[11], &kd, 4); // copy 4 bytes (from LE)
	if (_ZDT_compute_checksum(zdthdl, TxDataBuff, 15, &TxDataBuff[15])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	payload[0] = TxDataBuff[0];
	memcpy(&payload[1], &TxDataBuff[1], 7); // First payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[8], 7);// Second payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[15], 1);// Third payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 2, payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x4A && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x4C: Set Homing Parameters
 * Modify Origin Homing Parameters.
 */
ZDT_ReturnCode ZDT_cmd_edit_homing_parameters(ZDT_Handler *zdthdl, bool do_store, ZDT_HomingParams hom_params)
{
	uint8_t payload[8]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t TxDataBuff[19];
	/* Filling full payload array */
	TxDataBuff[0] = 0x4C; TxDataBuff[1] = 0xAE; // Fixed values
	TxDataBuff[2] = (uint8_t) do_store;
	TxDataBuff[3] = (uint8_t) hom_params.mode;
	TxDataBuff[4] = (uint8_t) hom_params.direction;
	cpy_le( &TxDataBuff[5], &(hom_params.speed), 2); // copy 2 bytes (from LE)
	cpy_le( &TxDataBuff[7], &(hom_params.timeout), 4); // copy 4 bytes (from LE)
	cpy_le( &TxDataBuff[11], &(hom_params.detection_speed), 2); // copy 2 bytes (from LE)
	cpy_le( &TxDataBuff[13], &(hom_params.detection_current), 2); // copy 2 bytes (from LE)
	cpy_le( &TxDataBuff[15], &(hom_params.detection_time), 2); // copy 2 bytes (from LE)
	TxDataBuff[17] = (uint8_t) hom_params.enable_pwr_on_auto_trigger;
	if (_ZDT_compute_checksum(zdthdl, TxDataBuff, 18, &TxDataBuff[18])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	payload[0] = TxDataBuff[0];
	memcpy(&payload[1], &TxDataBuff[1], 7); // First payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[8], 7);// Second payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[15], 4);// Third payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 2, payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x4C && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x4F: Allow /10 on Communication Control Speed commands
 * Modify Communication Control Input Speed to Reduce by 10x (S_Vel_IS Menu Option).
 */
ZDT_ReturnCode ZDT_cmd_allow_divide10_on_com_speed_cmd(ZDT_Handler *zdthdl, bool do_store, bool do_divide10)
{
	uint8_t payload[5]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x4F; payload[1] = 0x71; // Fixed values
	payload[2] = (uint8_t) do_store;
	payload[3] = (uint8_t) do_divide10;
	if (_ZDT_compute_checksum(zdthdl, payload, 4, &payload[4])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x4F && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x84: Edit Microstep Value
 * Modify Any Microstep
 */
ZDT_ReturnCode ZDT_cmd_edit_microstep(ZDT_Handler *zdthdl,  bool do_store, uint8_t microstep_value)
{
	uint8_t payload[5]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x84; payload[1] = 0x8A; // Fixed values
	payload[2] = (uint8_t) do_store;
	payload[3] = microstep_value;
	if (_ZDT_compute_checksum(zdthdl, payload, 4, &payload[4])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x84 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x93: Set 0 Position Homing
 * Set the Zero Position for Single-Turn Homing at current position.
 */
ZDT_ReturnCode ZDT_cmd_set_0_position_homing(ZDT_Handler *zdthdl, bool do_store)
{
	uint8_t payload[4]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x93; payload[1] = 0x88; // Fixed values
	payload[2] = (uint8_t) do_store;
	if (_ZDT_compute_checksum(zdthdl, payload, 3, &payload[3])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 4);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x93 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x9A: Trigger Homing
 * Trigger Homing.
 */
ZDT_ReturnCode ZDT_cmd_trigger_homing(ZDT_Handler *zdthdl, ZDT_HomMode hom_mode, bool m_m_flag)
{
	uint8_t payload[4]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x9A;  // Fixed values
	payload[1] = (uint8_t) hom_mode;
	payload[2] = (uint8_t) m_m_flag;
	if (_ZDT_compute_checksum(zdthdl, payload, 3, &payload[3])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 4);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0x9A) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0x9A && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0x9C: Interrupt Homing.
 * Force Interrupt and Exit Homing Operation.
 */
ZDT_ReturnCode ZDT_cmd_interrupt_homing(ZDT_Handler *zdthdl)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0x9C; payload[1] = 0x48; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0x9C && RxBuff[1] == 0xE2) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0x9C && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xAE: Edit ID Address
 * Edit ID Address of target device.
 */
ZDT_ReturnCode ZDT_cmd_edit_ID_address(ZDT_Handler *zdthdl, bool do_store, uint8_t new_mot_id)
{
	uint8_t payload[6]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0xAE; payload[1] = 0x4B; // Fixed values
	payload[2] = (uint8_t) do_store;
	payload[3] = new_mot_id;
	if (_ZDT_compute_checksum(zdthdl, payload, 4, &payload[4])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xAE) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xAE && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xF3: Motor Enable Control.
 * Enables control of the motor through CAN.
 */
ZDT_ReturnCode ZDT_cmd_motor_enable_control(ZDT_Handler *zdthdl, bool do_enable, bool m_m_flag)
{
	uint8_t payload[5]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0xF3; payload[1] = 0xAB; // Fixed values
	payload[2] = (uint8_t) do_enable;
	payload[3] = (uint8_t) m_m_flag;
	if (_ZDT_compute_checksum(zdthdl, payload, 4, &payload[4])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if (RxBuff[0] == 0xF3 && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xF6: Speed Mode Control
 * Speed Mode Control.
 */
ZDT_ReturnCode ZDT_cmd_speed_mode_control(ZDT_Handler *zdthdl, ZDT_Dir dir, uint16_t speed, uint8_t accel_level, bool m_m_flag)
{
	uint8_t payload[7]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0xF6; // Fixed values
	payload[1] = (uint8_t) dir;
	cpy_le( &payload[2], &speed, 2); // copy 2 bytes (from LE)
	payload[4] = accel_level;
	payload[5] = (uint8_t) m_m_flag;
	if (_ZDT_compute_checksum(zdthdl, payload, 6, &payload[6])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 7);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xF6) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xF6 && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xF7: Auto-Run on Power-On
 * Store a Set of Speed Mode Parameters (Direction, Speed, Acceleration), Auto-Run on Power-On
 */
ZDT_ReturnCode ZDT_cmd_auto_run_on_power_on(ZDT_Handler *zdthdl, ZDT_Dir dir, uint8_t store_clear_flag, uint16_t speed, uint8_t accel_level, bool do_en_pin_control)
{
	uint8_t payload[8]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t TxDataBuff[9];
	/* Filling full payload array */
	TxDataBuff[0] = 0xF7; TxDataBuff[1] = 0x1C; // Fixed values
	TxDataBuff[2] = store_clear_flag;
	TxDataBuff[3] = (uint8_t) dir;
	cpy_le( &TxDataBuff[4], &speed, 2); // copy 2 bytes (from LE)
	TxDataBuff[6] =  accel_level;
	TxDataBuff[7] = (uint8_t) do_en_pin_control;
	if (_ZDT_compute_checksum(zdthdl, TxDataBuff, 8, &TxDataBuff[8])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	payload[0] = TxDataBuff[0];
	memcpy(&payload[1], &TxDataBuff[1], 7); // First payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[8], 1);// Second payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, payload, 2);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xF7) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xF7 && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xFD: Position Mode Control
 * Send position-mode control command.
 */
ZDT_ReturnCode ZDT_cmd_position_mode_control(ZDT_Handler *zdthdl, ZDT_Dir dir, uint32_t pulse_count, uint16_t speed, uint8_t accel_level, bool is_relative, bool m_m_flag)
{
	uint8_t payload[8]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize; uint8_t TxDataBuff[12];
	/* Filling full payload array */
	TxDataBuff[0] = 0xFD; // Fixed values
	TxDataBuff[1] = (uint8_t) dir;
	cpy_le( &TxDataBuff[2], &speed, 2); // copy 2 bytes (from LE)
	TxDataBuff[4] = accel_level;
	cpy_le( &TxDataBuff[5], &pulse_count, 4); // copy 4 bytes (from LE)
	TxDataBuff[9] = (uint8_t) is_relative;
	TxDataBuff[10] = (uint8_t) m_m_flag;
	if (_ZDT_compute_checksum(zdthdl, TxDataBuff, 11, &TxDataBuff[11])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	payload[0] = TxDataBuff[0];
	memcpy(&payload[1], &TxDataBuff[1], 7); // First payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 8);
	if (ret) {return ZDT_ERROR;}
	memcpy(&payload[1], &TxDataBuff[8], 4);// Second payload
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8) + 1, payload, 5);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xFD) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xFD && (RxBuff[1] == 0x02 || RxBuff[1] == 0x9F) && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xFE: Immediate Stop
 * Stop the motor immediately.
 */
ZDT_ReturnCode ZDT_cmd_immediate_stop(ZDT_Handler *zdthdl, bool m_m_flag)
{
	uint8_t payload[4]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0xFE; payload[1] = 0x98; // Fixed values
	payload[2] = (uint8_t) m_m_flag;
	if (_ZDT_compute_checksum(zdthdl, payload, 3, &payload[3])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 4);
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xFE) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xFE && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Command 0xFF: Multi-Machine Synchronized Motion
 * Start Multi-Machine Synchronized Motion. Please refer to the documentation.
 */
ZDT_ReturnCode ZDT_cmd_M_M_synchronized_motion(ZDT_Handler *zdthdl, bool do_broadcast)
{
	uint8_t payload[3]; ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	payload[0] = 0xFF; payload[1] = 0x66; // Fixed values
	if (_ZDT_compute_checksum(zdthdl, payload, 2, &payload[2])) {return ZDT_ERROR;} // Append checksum value

	/* Send command */
	if (ZDT_CAN_clear_queues_of(zdthdl->mot_id)) {return ZDT_ERROR;} // Clear message queues used by this device
	if (do_broadcast)
	{
		ret = ZDT_CAN_send(zdthdl, 0x00, payload, 34); // CAN id override to broadcast
	} else {
		ret = ZDT_CAN_send(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), payload, 3);
	}
	if (ret) {return ZDT_ERROR;}

	/* Read return message(s) */
	ret = ZDT_CAN_receive_from(zdthdl, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xFF) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xFF && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

/*
 * Wait for "Reached" return message from the device ({0xFD, 0x9F, 0x--} message)
 * Wait for "Reached" return message from the device, after 0xFD Position Mode Control cmd.
 */
ZDT_ReturnCode ZDT_receive_reached_message(ZDT_Handler *zdthdl, uint32_t timeout)
{
	ZDT_ReturnCode ret; uint8_t RxBuff[8]; uint8_t RxSize;
	/* Read return message(s) */
	ret = ZDT_CAN_receive_from_timeout(zdthdl, timeout, (((uint32_t)(zdthdl->mot_id)) <<8), RxBuff, &RxSize);
	if (ret) {return ZDT_ERROR;}

	if (RxSize < 3) {return ZDT_ERROR;} // Too tiny
	if ((RxBuff[0] == 0x00) && (RxBuff[1] == 0xEE)) {return ZDT_ERROR;} // Manual check: Error return message
	if ((RxBuff[0] == 0xFF) && (RxBuff[1] == 0xE2)) {return ZDT_CONDITIONS_NOT_MET;} // Manual check: Condition not met return message
	if (RxBuff[0] == 0xFF && RxBuff[1] == 0x02 && _ZDT_check_checksum(zdthdl, RxBuff, 3))
	{
		return ZDT_OK;
	} else {
		return ZDT_ERROR;
	}
}

