#ifndef __FASHION_STAR_UART_SERVO_H
#define __FASHION_STAR_UART_SERVO_H

#include "user_uart.h" // 引入用户串口层
#include <stdint.h>
#include <math.h>

/* ============================ 宏定义 ============================ */

// 舵机指令ID
#define FSUS_CMD_PING 0x01                                  // 舵机通讯检测
#define FSUS_CMD_RESET_USER_DATA 0x02                       // 重置用户数据
#define FSUS_CMD_READ_DATA 0x03                             // 读取数据
#define FSUS_CMD_WRITE_DATA 0x04                            // 写入数据
#define FSUS_CMD_READ_BATCH_DATA 0x05                       // 读取批次数据
#define FSUS_CMD_WRITE_BATCH_DATA 0x06                      // 写入批次资料
#define FSUS_CMD_ROTATE 0x08                                // 角度模式控制
#define FSUS_CMD_DAMPING 0x09                               // 阻尼模式控制
#define FSUS_CMD_READ_ANGLE 0x0A                            // 读取角度
#define FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL 0x0B           // 角度模式控制(基于加减速时间)
#define FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY 0x0C           // 角度模式控制(基于速率)
#define FSUS_CMD_SET_SERVO_ANGLE_MTURN 0x0D                 // 多圈角度模式控制
#define FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL 0x0E     // 多圈角度模式控制(基于加减速时间)
#define FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY 0x0F     // 多圈角度模式控制(基于速率)
#define FSUS_CMD_QUERY_SERVO_ANGLE_MTURN 0x10               // 读取多圈角度
#define FSUS_CMD_RESERT_SERVO_ANGLE_MTURN 0x11              // 重设多圈角度
#define FSUS_CMD_BEGIN_ASYNC 0x12                           // 开始异步命令
#define FSUS_CMD_END_ASYNC 0x13                             // 结束异步命令
#define FSUS_CMD_SET_SERVO_ReadData 0x16                    // 舵机数据监控
#define FSUS_CMD_SET_ORIGIN_POINT 0x17                      // 设置原点
#define FSUS_CMD_CONTROL_MODE_STOP 0x18                     // 控制模式停止指令
#define FSUS_CMD_SET_SERVO_SyncCommand 0x19                 // 同步命令
#define FSUS_CMD_NUM 0x20                                   // 指令ID总数

// 帧头
#define FSUS_PACK_REQUEST_HEADER 0x4C12  // 请求包帧头
#define FSUS_PACK_RESPONSE_HEADER 0x1C05 // 响应包帧头

// 接收状态
#define FSUS_RECV_FLAG_HEADER (1 << 0)   // 帧头
#define FSUS_RECV_FLAG_CMD_ID (1 << 1)   // 指令ID
#define FSUS_RECV_FLAG_SIZE (1 << 2)     // 长度
#define FSUS_RECV_FLAG_CONTENT (1 << 3)  // 内容
#define FSUS_RECV_FLAG_CHECKSUM (1 << 4) // 校验和

// 缓冲区大小
#define FSUS_PACK_RESPONSE_MAX_SIZE 255
#define FSUS_TIMEOUT_MS 100 // 接收超时时间

// 同步命令模式选择
#define MODE_SET_SERVO_ANGLE 1
#define MODE_SET_SERVO_ANGLE_BY_INTERVAL 2
#define MODE_SET_SERVO_ANGLE_BY_VELOCITY 3
#define MODE_SET_SERVO_ANGLE_MTURN 4
#define MODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL 5
#define MODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY 6
#define MODE_Query_SERVO_Monitor 7

/* ============================ 枚举和结构体 ============================ */

// 状态码
typedef enum
{
    FSUS_STATUS_SUCCESS = 0,             // 成功
    FSUS_STATUS_FAIL = 1,                // 失败
    FSUS_STATUS_TIMEOUT = 2,             // 超时
    FSUS_STATUS_WRONG_RESPONSE_HEADER = 3, // 错误响应帧头
    FSUS_STATUS_UNKOWN_CMD_ID = 4,       // 未知指令ID
    FSUS_STATUS_SIZE_TOO_BIG = 5,        // 长度过长
    FSUS_STATUS_CHECKSUM_ERROR = 6,      // 校验和错误
    FSUS_STATUS_ID_NOT_MATCH = 7,        // 舵机ID不匹配
    FSUS_STATUS_ERRO = 8,                // 舵机故障
    // --- 以下为新增的状态码 ---
    FSUS_STATUS_WRONG_RESPONSE_CMD = 9,  // 收到的回应包指令与请求不符
    FSUS_STATUS_INVALID_PARAM = 10,      // 无效的函数参数
} FSUS_STATUS;

// 舵机数据包
typedef struct
{
    uint16_t header;                                // 帧头
    uint8_t cmdId;                                  // 指令ID
    uint16_t size;                                  // 长度
    uint8_t content[FSUS_PACK_RESPONSE_MAX_SIZE]; // 内容
    uint8_t checksum;                               // 校验和
    uint8_t status;                                 // 接收状态
    uint8_t isSync;                                 // 是否为同步/扩展包
} PackageTypeDef;

// 舵机数据监控结构体 (用于接收数据)
typedef struct
{
    uint8_t id;          // 舵机ID
    int16_t voltage;     // 电压 (mV)
    int16_t current;     // 电流 (mA)
    int16_t power;       // 功率 (mW)
    float temperature;   // 温度 (°C)
    uint8_t status;      // 状态
    float angle;         // 角度 (°)
    int16_t circle_count; // 圈数
} ServoData;

// 同步控制结构体 (用于发送指令)
typedef struct
{
    uint8_t id;              // 舵机ID
    float angle;             // 角度
    float velocity;          // 速度
    uint16_t interval_single; // 单圈时间(ms)
    uint32_t interval_multi; // 多圈时间(ms)
    uint16_t t_acc;           // 加速时间(ms)
    uint16_t t_dec;           // 减速时间(ms)
    uint16_t power;           // 功率(mW)
} FSUS_sync_servo;

/* ============================ 函数声明 ============================ */

// 底层协议收发
void FSUS_Package2RingBuffer(PackageTypeDef *pkg, RingBufferTypeDef *ringBuf);
uint8_t FSUS_CalcChecksum(PackageTypeDef *pkg);
FSUS_STATUS FSUS_IsValidResponsePackage(PackageTypeDef *pkg);
FSUS_STATUS FSUS_RingBuffer2Package(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg);
void FSUS_SendPackage_Common(Usart_DataTypeDef *usart, uint8_t cmdId, uint16_t size, uint8_t *content, uint8_t isSync);
FSUS_STATUS FSUS_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg);

// 单舵机控制指令
FSUS_STATUS FSUS_Ping(Usart_DataTypeDef *usart, uint8_t servo_id);
FSUS_STATUS FSUS_ResetUserData(Usart_DataTypeDef *usart, uint8_t servo_id);
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t *size);
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size);
FSUS_STATUS FSUS_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t power);
FSUS_STATUS FSUS_SetServoAngleByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power);
FSUS_STATUS FSUS_SetServoAngleByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, float velocity, uint16_t t_acc, uint16_t t_dec, uint16_t power);
FSUS_STATUS FSUS_QueryServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle);
FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint32_t interval, uint16_t power);
FSUS_STATUS FSUS_SetServoAngleMTurnByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint32_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power);
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, float velocity, uint16_t t_acc, uint16_t t_dec, uint16_t power);
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle);
FSUS_STATUS FSUS_DampingMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint16_t power);
FSUS_STATUS FSUS_ServoAngleReset(Usart_DataTypeDef *usart, uint8_t servo_id);
FSUS_STATUS FSUS_SetOriginPoint(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t reset);
FSUS_STATUS FSUS_BeginAsync(Usart_DataTypeDef *usart);
FSUS_STATUS FSUS_EndAsync(Usart_DataTypeDef *usart, uint8_t cancel);
FSUS_STATUS FSUS_ServoMonitor(Usart_DataTypeDef *usart, uint8_t servo_id, ServoData *servodata);
FSUS_STATUS FSUS_StopOnControlMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t mode, uint16_t power);

// 同步控制指令 (注意函数签名的变化)
FSUS_STATUS FSUS_SyncCommand(Usart_DataTypeDef *usart, uint8_t servo_count, uint8_t ServoMode, FSUS_sync_servo servoSync[], ServoData servo_data_out[]);
FSUS_STATUS FSUS_SyncServoMonitor(Usart_DataTypeDef *usart, uint8_t servo_count, ServoData servo_data_out[]);

#endif // __FASHION_STAR_UART_SERVO_H