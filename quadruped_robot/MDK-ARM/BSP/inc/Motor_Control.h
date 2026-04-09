#ifndef __MOTOR_CONTRROL_H
#define __MOTOR_CONTRROL_H


#include "mytype.h"
#include "can.h"

#define CHASSIS_CAN1 hcan1
#define CHASSIS_CAN2 hcan2

// RM motor CAN IDs
#define Motor_1_ID	0x201 
#define Motor_2_ID	0x202 
#define Motor_3_ID	0x203 
#define Motor_4_ID	0x204 
#define Motor_5_ID	0x205 
#define Motor_6_ID	0x206 
#define Motor_7_ID	0x207 
#define Motor_8_ID	0x208

#define Can1_Send_ID 0x200
#define Can2_Send_ID 0x1FF

#define FILTER_BUF_LEN		5

typedef  enum
{
	IDLE,
	PIDSPEED,
	PIDPOSITION,
	MOTOR_ERROR,
	MOTOR_PWM,   // PWM output mode
	MOTOR_CURRENT,
	MOTOR_BUFFER,
}DriverState;// System state

/* Motor feedback and command state structure */
typedef struct 
{
	float PositionExpected;
	float PositionMeasure;
	float SpeedExpected;
	float SpeedMeasure;

	float PosPre;
	float PosNow;
	float CurrentExpected;
	float CurrentMeasure;
	
	int32_t		round_cnt;
	uint16_t	offset_angle;
	int32_t		total_angle;
	
	int32_t 	PWM;
  DriverState    State;	
}MotorTypeDef;

typedef enum {
   CAN_PACKET_SET_DUTY = 0,
   CAN_PACKET_SET_CURRENT,
   CAN_PACKET_SET_CURRENT_BRAKE,
   CAN_PACKET_SET_RPM,
   CAN_PACKET_SET_POS,
   CAN_PACKET_FILL_RX_BUFFER,
   CAN_PACKET_FILL_RX_BUFFER_LONG,
   CAN_PACKET_PROCESS_RX_BUFFER,
   CAN_PACKET_PROCESS_SHORT_BUFFER,
   CAN_PACKET_STATUS,
   CAN_PACKET_SET_CURRENT_REL,
   CAN_PACKET_SET_CURRENT_BRAKE_REL,
	 CAN_PACKET_SET_CURRENT_HANDBRAKE,
	 CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
   CAN_PACKET_GET_STATUS
} CAN_PACKET_ID;

// DC motor control
/************************* Defines *************************/
#define DCMOTOR1_PIN1            GPIO_PIN_0    		              // DCMOTOR1 direction pin 1
#define DCMOTOR1_PIN2            GPIO_PIN_1  
#define DCMOTOR2_PIN1            GPIO_PIN_4    		              // DCMOTOR2 direction pin 1
#define DCMOTOR2_PIN2            GPIO_PIN_5  
#define DCMOTOR3_PIN1            GPIO_PIN_12    		              // DCMOTOR3 direction pin 1
#define DCMOTOR3_PIN2            GPIO_PIN_6

#define DCMOTOR4_PIN1            GPIO_PIN_0    		              // DCMOTOR4 direction pin 1
#define DCMOTOR4_PIN2            GPIO_PIN_4  
#define DCMOTOR5_PIN1            GPIO_PIN_1    		              // DCMOTOR5 direction pin 1
#define DCMOTOR5_PIN2            GPIO_PIN_5  
#define DCMOTOR6_PIN1            GPIO_PIN_4    		              // DCMOTOR6 direction pin 1
#define DCMOTOR6_PIN2            GPIO_PIN_5 

#define DCMOTOR1_PORT            GPIOF                  					// DCMOTOR1 GPIO port
#define DCMOTOR2_PORT            GPIOE                  					// DCMOTOR2 GPIO port
#define DCMOTOR3_PORT            GPIOE                  					// DCMOTOR3 GPIO port

#define DCMOTOR4_PORT            GPIOC                 					// DCMOTOR4 GPIO port
#define DCMOTOR5_PORT            GPIOC                  					// DCMOTOR5 GPIO port
#define DCMOTOR6_PORT            GPIOA                  					// DCMOTOR6 GPIO port

/*---------------------- DC motor direction macros ------------------------*/
					
#define DCMOTOR1_FW 	  {	HAL_GPIO_WritePin(DCMOTOR1_PORT, DCMOTOR1_PIN1, GPIO_PIN_SET);HAL_GPIO_WritePin(DCMOTOR1_PORT,DCMOTOR1_PIN2,GPIO_PIN_RESET);}	// Motor forward
#define DCMOTOR1_RB 	  {	HAL_GPIO_WritePin(DCMOTOR1_PORT, DCMOTOR1_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR1_PORT,DCMOTOR1_PIN2,GPIO_PIN_SET);}  // Motor reverse
#define DCMOTOR1_ST 	  { HAL_GPIO_WritePin(DCMOTOR1_PORT, DCMOTOR1_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR1_PORT,DCMOTOR1_PIN2,GPIO_PIN_RESET);}  // Motor stop

#define DCMOTOR2_FW 	  {	HAL_GPIO_WritePin(DCMOTOR2_PORT, DCMOTOR2_PIN1, GPIO_PIN_SET);HAL_GPIO_WritePin(DCMOTOR2_PORT,DCMOTOR2_PIN2,GPIO_PIN_RESET);}	// Motor forward
#define DCMOTOR2_RB 	  {	HAL_GPIO_WritePin(DCMOTOR2_PORT, DCMOTOR2_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR2_PORT,DCMOTOR2_PIN2,GPIO_PIN_SET);}  // Motor reverse
#define DCMOTOR2_ST 	  { HAL_GPIO_WritePin(DCMOTOR2_PORT, DCMOTOR2_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR2_PORT,DCMOTOR2_PIN2,GPIO_PIN_RESET);}  // Motor stop

#define DCMOTOR3_FW 	  {	HAL_GPIO_WritePin(DCMOTOR3_PORT, DCMOTOR3_PIN1, GPIO_PIN_SET);HAL_GPIO_WritePin(DCMOTOR3_PORT,DCMOTOR3_PIN2,GPIO_PIN_RESET);}	// Motor forward
#define DCMOTOR3_RB 	  {	HAL_GPIO_WritePin(DCMOTOR3_PORT, DCMOTOR3_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR3_PORT,DCMOTOR3_PIN2,GPIO_PIN_SET);}  // Motor reverse
#define DCMOTOR3_ST 	  { HAL_GPIO_WritePin(DCMOTOR3_PORT, DCMOTOR3_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR3_PORT,DCMOTOR3_PIN2,GPIO_PIN_RESET);}  // Motor stop

#define DCMOTOR4_FW 	  {	HAL_GPIO_WritePin(DCMOTOR4_PORT, DCMOTOR4_PIN1, GPIO_PIN_SET);HAL_GPIO_WritePin(DCMOTOR4_PORT,DCMOTOR4_PIN2,GPIO_PIN_RESET);}	// Motor forward
#define DCMOTOR4_RB 	  {	HAL_GPIO_WritePin(DCMOTOR4_PORT, DCMOTOR4_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTR4_PORT,DCMOTOR4_PIN2,GPIO_PIN_SET);}  // Motor reverse
#define DCMOTOR4_ST 	  { HAL_GPIO_WritePin(DCMOTOR4_PORT, DCMOTOR4_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR4_PORT,DCMOTOR4_PIN2,GPIO_PIN_RESET);}  // Motor stop


#define DCMOTOR5_FW 	  {	HAL_GPIO_WritePin(DCMOTOR5_PORT, DCMOTOR5_PIN1, GPIO_PIN_SET);HAL_GPIO_WritePin(DCMOTOR5_PORT,DCMOTOR5_PIN2,GPIO_PIN_RESET);}	// Motor forward
#define DCMOTOR5_RB 	  {	HAL_GPIO_WritePin(DCMOTOR5_PORT, DCMOTOR5_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR5_PORT,DCMOTOR5_PIN2,GPIO_PIN_SET);}  // Motor reverse
#define DCMOTOR5_ST 	  { HAL_GPIO_WritePin(DCMOTOR5_PORT, DCMOTOR5_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR5_PORT,DCMOTOR5_PIN2,GPIO_PIN_RESET);}  // Motor stop


#define DCMOTOR6_FW 	  {	HAL_GPIO_WritePin(DCMOTOR6_PORT, DCMOTOR6_PIN1, GPIO_PIN_SET);HAL_GPIO_WritePin(DCMOTOR6_PORT,DCMOTOR6_PIN2,GPIO_PIN_RESET);}	// Motor forward
#define DCMOTOR6_RB 	  {	HAL_GPIO_WritePin(DCMOTOR6_PORT, DCMOTOR6_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR6_PORT,DCMOTOR6_PIN2,GPIO_PIN_SET);}  // Motor reverse
#define DCMOTOR6_ST 	  { HAL_GPIO_WritePin(DCMOTOR6_PORT, DCMOTOR6_PIN1, GPIO_PIN_RESET);HAL_GPIO_WritePin(DCMOTOR6_PORT,DCMOTOR6_PIN2,GPIO_PIN_RESET);}  // Motor stop

extern MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
extern MotorTypeDef Motor_5, Motor_6,Motor_7, Motor_8;

extern uint32_t ccr1[6]; // DC motor PWM compare values

extern MotorTypeDef* motors_2006[3];
extern MotorTypeDef* motors_3508[3];  // RM motors


void get_moto_offset(MotorTypeDef *ptr, CAN_HandleTypeDef* hcan);
void get_total_angle(MotorTypeDef *p);
void CAN1_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void RMMotor_Init(void);


#endif
