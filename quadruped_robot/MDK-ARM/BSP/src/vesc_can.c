#include "vesc_can.h"

#include <string.h>

float Mad_Velocity[VESC_CAN_DEVICE_COUNT] = {0.0f};
float Vesc_Feedback_Array[VESC_FEEDBACK_ARRAY_SIZE] = {0.0f};

static VescCanStatusTypeDef vesc_status[VESC_CAN_DEVICE_COUNT];
static VescCanStatusTypeDef *vesc_can_status_from_id(uint8_t controller_id);

static void vesc_can_update_feedback_array(uint8_t controller_id)
{
	uint8_t base_index;
	VescCanStatusTypeDef *status = vesc_can_status_from_id(controller_id);

	if (status == 0) {
		return;
	}

	base_index = (uint8_t)((controller_id - 1U) * 2U);
	Vesc_Feedback_Array[base_index] = status->rpm;
	Vesc_Feedback_Array[base_index + 1U] = status->motor_current;
}

static bool vesc_can_is_valid_id(uint8_t controller_id)
{
	return controller_id >= 1U && controller_id <= VESC_CAN_DEVICE_COUNT;
}

static VescCanStatusTypeDef *vesc_can_status_from_id(uint8_t controller_id)
{
	if (!vesc_can_is_valid_id(controller_id)) {
		return 0;
	}

	return &vesc_status[controller_id - 1U];
}

static int16_t vesc_can_buffer_get_int16(const uint8_t *buffer, uint8_t *index)
{
	int16_t value = (int16_t)(((uint16_t)buffer[*index] << 8) | buffer[*index + 1U]);
	*index += 2U;
	return value;
}

static int32_t vesc_can_buffer_get_int32(const uint8_t *buffer, uint8_t *index)
{
	int32_t value = ((int32_t)buffer[*index] << 24)
			| ((int32_t)buffer[*index + 1U] << 16)
			| ((int32_t)buffer[*index + 2U] << 8)
			| (int32_t)buffer[*index + 3U];
	*index += 4U;
	return value;
}

static void vesc_can_buffer_append_int32(uint8_t *buffer, int32_t number, uint8_t *index)
{
	buffer[(*index)++] = (uint8_t)(number >> 24);
	buffer[(*index)++] = (uint8_t)(number >> 16);
	buffer[(*index)++] = (uint8_t)(number >> 8);
	buffer[(*index)++] = (uint8_t)number;
}

void vesc_can_init(void)
{
	uint8_t index;

	memset(vesc_status, 0, sizeof(vesc_status));
	for (index = 0U; index < VESC_CAN_DEVICE_COUNT; index++) {
		vesc_status[index].id = index + 1U;
	}
	memset(Mad_Velocity, 0, sizeof(Mad_Velocity));
	memset(Vesc_Feedback_Array, 0, sizeof(Vesc_Feedback_Array));
}

HAL_StatusTypeDef vesc_can_set_rpm(uint8_t controller_id, float rpm)
{
	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox;
	uint8_t tx_data[4];
	uint8_t send_index = 0U;

	if (!vesc_can_is_valid_id(controller_id)) {
		return HAL_ERROR;
	}

	tx_header.StdId = 0U;
	tx_header.ExtId = controller_id | ((uint32_t)VESC_CAN_PACKET_SET_RPM << 8);
	tx_header.IDE = CAN_ID_EXT;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 4U;
	tx_header.TransmitGlobalTime = DISABLE;

	vesc_can_buffer_append_int32(tx_data, (int32_t)rpm, &send_index);

	return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

void vesc_can_handle_rx_frame(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *data)
{
	VescCanStatusTypeDef *status;
	uint8_t controller_id;
	uint8_t packet_id;
	uint8_t index = 0U;

	if (rx_header == 0 || data == 0 || rx_header->IDE != CAN_ID_EXT) {
		return;
	}

	controller_id = (uint8_t)(rx_header->ExtId & 0xFFU);
	packet_id = (uint8_t)((rx_header->ExtId >> 8) & 0xFFU);
	status = vesc_can_status_from_id(controller_id);
	if (status == 0) {
		return;
	}

	status->online = true;
	status->last_update_ms = HAL_GetTick();

	switch ((VescCanPacketId)packet_id) {
	case VESC_CAN_PACKET_STATUS:
		status->rpm = (float)vesc_can_buffer_get_int32(data, &index);
		status->motor_current = (float)vesc_can_buffer_get_int16(data, &index) / 10.0f;
		status->duty_cycle = (float)vesc_can_buffer_get_int16(data, &index) / 1000.0f;
		status->received_mask |= 0x01U;
		vesc_can_update_feedback_array(controller_id);
		break;

	case VESC_CAN_PACKET_STATUS_2:
		status->amp_hours = (float)vesc_can_buffer_get_int32(data, &index) / 10000.0f;
		status->amp_hours_charged = (float)vesc_can_buffer_get_int32(data, &index) / 10000.0f;
		status->received_mask |= 0x02U;
		break;

	case VESC_CAN_PACKET_STATUS_3:
		status->watt_hours = (float)vesc_can_buffer_get_int32(data, &index) / 10000.0f;
		status->watt_hours_charged = (float)vesc_can_buffer_get_int32(data, &index) / 10000.0f;
		status->received_mask |= 0x04U;
		break;

	case VESC_CAN_PACKET_STATUS_4:
		status->temp_fet = (float)vesc_can_buffer_get_int16(data, &index) / 10.0f;
		status->temp_motor = (float)vesc_can_buffer_get_int16(data, &index) / 10.0f;
		status->input_current = (float)vesc_can_buffer_get_int16(data, &index) / 10.0f;
		status->pid_pos_now = (float)vesc_can_buffer_get_int16(data, &index) / 50.0f;
		status->received_mask |= 0x08U;
		break;

	case VESC_CAN_PACKET_STATUS_5:
		status->tachometer = vesc_can_buffer_get_int32(data, &index);
		status->v_in = (float)vesc_can_buffer_get_int16(data, &index) / 10.0f;
		status->received_mask |= 0x10U;
		break;

	default:
		break;
	}
}

const VescCanStatusTypeDef *vesc_can_get_status(uint8_t controller_id)
{
	return vesc_can_status_from_id(controller_id);
}

bool vesc_can_is_status_online(uint8_t controller_id, uint32_t timeout_ms)
{
	const VescCanStatusTypeDef *status = vesc_can_get_status(controller_id);

	if (status == 0 || !status->online) {
		return false;
	}

	return (HAL_GetTick() - status->last_update_ms) <= timeout_ms;
}

const float *vesc_can_get_feedback_array(void)
{
	return Vesc_Feedback_Array;
}
