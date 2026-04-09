#ifndef __VESC_CAN_H
#define __VESC_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#define VESC_CAN_DEVICE_COUNT 4U
#define VESC_FEEDBACK_ARRAY_SIZE (VESC_CAN_DEVICE_COUNT * 2U)

typedef enum {
	VESC_CAN_PACKET_SET_DUTY = 0,
	VESC_CAN_PACKET_SET_CURRENT = 1,
	VESC_CAN_PACKET_SET_CURRENT_BRAKE = 2,
	VESC_CAN_PACKET_SET_RPM = 3,
	VESC_CAN_PACKET_SET_POS = 4,
	VESC_CAN_PACKET_FILL_RX_BUFFER = 5,
	VESC_CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
	VESC_CAN_PACKET_PROCESS_RX_BUFFER = 7,
	VESC_CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
	VESC_CAN_PACKET_STATUS = 9,
	VESC_CAN_PACKET_SET_CURRENT_REL = 10,
	VESC_CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
	VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
	VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
	VESC_CAN_PACKET_STATUS_2 = 14,
	VESC_CAN_PACKET_STATUS_3 = 15,
	VESC_CAN_PACKET_STATUS_4 = 16,
	VESC_CAN_PACKET_STATUS_5 = 27,
	VESC_CAN_PACKET_STATUS_6 = 58,
} VescCanPacketId;

typedef struct {
	uint8_t id;
	bool online;
	uint32_t last_update_ms;
	uint8_t received_mask;
	float rpm;
	float motor_current;
	float duty_cycle;
	float amp_hours;
	float amp_hours_charged;
	float watt_hours;
	float watt_hours_charged;
	float temp_fet;
	float temp_motor;
	float input_current;
	float pid_pos_now;
	float v_in;
	int32_t tachometer;
} VescCanStatusTypeDef;

extern float Mad_Velocity[VESC_CAN_DEVICE_COUNT];
extern float Vesc_Feedback_Array[VESC_FEEDBACK_ARRAY_SIZE];

void vesc_can_init(void);
HAL_StatusTypeDef vesc_can_set_rpm(uint8_t controller_id, float rpm);
void vesc_can_handle_rx_frame(const CAN_RxHeaderTypeDef *rx_header, const uint8_t *data);
const VescCanStatusTypeDef *vesc_can_get_status(uint8_t controller_id);
bool vesc_can_is_status_online(uint8_t controller_id, uint32_t timeout_ms);
const float *vesc_can_get_feedback_array(void);

#endif