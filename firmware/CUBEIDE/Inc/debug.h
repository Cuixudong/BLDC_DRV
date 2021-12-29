/*
 * debug.h
 *
 *  Created on: 2021年12月29日
 *      Author: 27731
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "main.h"

#define EN_CRC            1

typedef enum
{
    DC_MOTOR        = 0x10, /* 直流有刷电机 */
    BLDC_MOTOR      = 0x11, /* 直流无刷电机 */
    PMSM_MOTOR      = 0x12, /* 永磁同步电机 */
    STEP_MOTOR      = 0x13, /* 步进电机 */
    SERVO_MOTOR     = 0x14, /* 伺服电机 */
    EXCHANG_MOTOR   = 0x15, /* 变频器(三相交流异步电机) */
    HELM_MOTOR      = 0x16, /* 舵机 */
}motor_code;

typedef enum
{
    IDLE_STATE     = 0x00, /* 空闲状态 */
    RUN_STATE      = 0x01, /* 运行状态 */
    ERROR_STATE    = 0x02, /* 错误状态 */
    LRTOUT_STATE   = 0x03, /* 堵转超时 */
    BREAKED_STATE  = 0x04, /* 刹车 */
}motor_state;

typedef enum
{
    HALT_CODE = 0x01, /* 停机 */
    RUN_CODE  = 0x02, /* 运行 */
    BREAKED   = 0x03, /* 刹车 */
}cmd_code;

#define DEBUG_REV_MAX_LEN 17
#define MAX_UPLOAD_LEN 50

#define DEBUG_DATA_HEAD 0xC5
#define DEBUG_DATA_END 0x5C

typedef struct
{
    uint8_t Ctrl_code;
    uint8_t Ctrl_mode;
    float *speed;
    float *torque;
    float pid[3];
}debug_data_rev;

#if 0
typedef enum
{
    STATUS_IDLE = 0x10,
    STATUS_HEAD = 0x11,
    STATUS_REVING = 0x12,
    STATUS_OVER = 0x13,
    STATUS_ERROR = 0x14,
}debug_rev_status;
#endif

typedef enum
{
    CMD_GET_STATUS      = 0x10,
    CMD_GET_SPEED       = 0x11,
    CMD_GET_HALL_ENC    = 0x12,
    CMD_GET_VBUS        = 0x13,
    CMD_GET_AMP         = 0x14,
    CMD_GET_TEMP        = 0x15,
    CMD_GET_SUM_LEN     = 0x16,
    CMD_GET_BEM         = 0x17,
    CMD_GET_MOTOR_CODE  = 0x18,
    CMD_GET_PID         = 0x20,
    CMD_SET_CTR_CODE    = 0x21,
    CMD_SET_CTR_MODE    = 0x22,
    CMD_SET_SPEED       = 0x23,
    CMD_SET_TORQUE      = 0x24,
    CMD_SET_PID1        = 0x31,
    CMD_SET_PID2        = 0x32,
    CMD_SET_PID3        = 0x33,
    CMD_SET_PID4        = 0x34,
    CMD_SET_PID5        = 0x35,
    CMD_SET_PID6        = 0x36,
    CMD_SET_PID7        = 0x37,
    CMD_SET_PID8        = 0x38,
    CMD_SET_PID9        = 0x39,
    CMD_SET_PID10       = 0x3A,
}cmd_type;

typedef enum
{
    TYPE_STATUS = 0x10,
    TYPE_SPEED = 0x11,
    TYPE_HAL_ENC = 0x12,
    TYPE_VBUS = 0x13,
    TYPE_AMP = 0x14,
    TYPE_TEMP = 0x15,
    TYPE_SUM_LEN = 0x16,
    TYPE_BEM = 0x17,
    TYPE_MOTOR_CODE = 0x18,
    TYPE_TORQUE = 0x19,
    TYPE_POWER = 0x1A,

    TYPE_PID1 = 0x20,
    TYPE_PID2 = 0x21,
    TYPE_PID3 = 0x22,
    TYPE_PID4 = 0x23,
    TYPE_PID5 = 0x24,
    TYPE_PID6 = 0x25,
    TYPE_PID7 = 0x26,
    TYPE_PID8 = 0x27,
    TYPE_PID9 = 0x28,
    TYPE_PID10 = 0x29,

    TYPE_USER_DATA = 0x30,
}upload_type;

typedef struct
{
    union
    {
        float pidf[3];
        int8_t pidi8[12];
    }pid;
}pid_struct;

typedef struct
{
    uint8_t status;
    int16_t speed;
    uint8_t hall_p;
    uint16_t encode_p;
    float bus_vol;
    float amp[3];
    float temp[2];
    uint64_t sum_len;
    float bem[3];
    uint8_t motor_code;
    float torque;
    float power;
    pid_struct pid[10];
    int16_t user_data[16];
}debug_data;

extern uint32_t g_pid_flag;

extern debug_data g_debug;//发送变量
extern debug_data_rev debug_rev;//接收变量

/******************************用户应用层*********************************/
/*驱动底层*/
void debug_handle(uint8_t *data);
void debug_obj_init(debug_data *data);
void debug_upload_data(debug_data * data, uint8_t upload_type);
/*用户应用层*/
void debug_init(void);
void debug_send_initdata(upload_type PIDx,float *SetPoint,float P,float I,float D);
void debug_send_current(float U_I,float V_I,float W_I);
void debug_send_valtage(float valtage);
void debug_send_power(float power);
void debug_send_speed(float speed);
void debug_send_distance(uint64_t len);
void debug_send_temp(float motor_temp,float board_temp);
void debug_send_motorstate(motor_state motor_codestae);
void debug_send_motorcode(motor_code motorcode);
void debug_receive_pid(upload_type PIDx,float *P,float *I,float *D);
uint8_t debug_receive_ctrl_code(void);
void debug_send_wave_data(uint8_t chx,int16_t wave);
//void debug_set_point_range(float max_limit,float min_limit);
void debug_set_point_range(float max_limit,float min_limit,float step_max);
void debug_updata_motorstate(int16_t speed,uint8_t flag_state);

#endif /* DEBUG_H_ */
