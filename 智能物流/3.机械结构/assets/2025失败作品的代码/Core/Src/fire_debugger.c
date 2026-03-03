#include "fire_debugger.h"
#include "Motor_PID.h" // 包含此文件以访问和修改PID参数
#include <string.h>   // 用于 memcpy

/* ============================================================================ */
/*                             协议相关宏定义                                   */
/* ============================================================================ */
#define RX_BUFFER_SIZE  128     // 接收环形缓冲区大小

// 协议帧头和功能码 (与知乎协议一致)
#define FRAME_HEADER    0xA5    // 帧头
#define CMD_SET_PID     0x01    // 设置PID参数命令
#define CMD_SET_TARGET  0x02    // 设置目标值命令 (此功能暂不实现)
#define CMD_WAVE_DATA   0x03    // 发送波形数据

/* ============================================================================ */
/*                                 私有变量                                     */
/* ============================================================================ */

extern Motor_PID_t MotorA_PID;
extern Motor_PID_t MotorB_PID;
extern Motor_PID_t MotorC_PID;
extern Motor_PID_t MotorD_PID;

static UART_HandleTypeDef *debugger_huart;

static uint8_t Rx_Buffer[RX_BUFFER_SIZE];
static uint16_t Rx_Write_Index = 0;
static uint16_t Rx_Read_Index = 0;

/* ============================================================================ */
/*                             私有函数声明                                     */
/* ============================================================================ */

static void Debugger_Parse_Data(const uint8_t *data, uint8_t len);

/* ============================================================================ */
/*                             公共函数实现                                     */
/* ============================================================================ */

void Debugger_Init(UART_HandleTypeDef *huart)
{
    debugger_huart = huart;
}

void Debugger_Receive_Handler(uint8_t RxData)
{
    Rx_Buffer[Rx_Write_Index] = RxData;
    Rx_Write_Index = (Rx_Write_Index + 1) % RX_BUFFER_SIZE;
}

void Debugger_Task(void)
{
    uint8_t temp_data[32];
    uint8_t data_len = 0;
    uint8_t func_code = 0;
    uint8_t checksum = 0;

    while (Rx_Read_Index != Rx_Write_Index)
    {
        if (Rx_Buffer[Rx_Read_Index] == FRAME_HEADER)
        {
            if ((Rx_Write_Index - Rx_Read_Index + RX_BUFFER_SIZE) % RX_BUFFER_SIZE > 3)
            {
                func_code = Rx_Buffer[(Rx_Read_Index + 1) % RX_BUFFER_SIZE];
                data_len = Rx_Buffer[(Rx_Read_Index + 2) % RX_BUFFER_SIZE];
                
                // <<< MODIFIED: 检查完整包长度，现在是 4 + data_len
                if ((Rx_Write_Index - Rx_Read_Index + RX_BUFFER_SIZE) % RX_BUFFER_SIZE >= (data_len + 4))
                {
                    // <<< MODIFIED: 复制 data_len + 4 字节
                    for (int i = 0; i < data_len + 4; i++)
                    {
                        temp_data[i] = Rx_Buffer[(Rx_Read_Index + i) % RX_BUFFER_SIZE];
                    }
                    
                    // <<< MODIFIED: 计算并验证校验和
                    checksum = 0;
                    for(int i = 1; i < data_len + 3; i++) // 从功能码开始累加
                    {
                        checksum += temp_data[i];
                    }
                    
                    if (checksum == temp_data[data_len + 3]) // 校验通过
                    {
                        Debugger_Parse_Data(temp_data, data_len + 4);
                    }

                    Rx_Read_Index = (Rx_Read_Index + data_len + 4) % RX_BUFFER_SIZE;
                    continue;
                }
                else
                {
                    break; 
                }
            }
        }
        Rx_Read_Index = (Rx_Read_Index + 1) % RX_BUFFER_SIZE;
    }
}


void Debugger_Send_Wave(float ch1, float ch2, float ch3, float ch4)
{
    uint8_t send_buf[21]; // 1(头)+1(码)+1(长)+16(数据)+1(校验)=20
    uint8_t i = 0;
    uint8_t checksum = 0;
    
    send_buf[i++] = FRAME_HEADER;           // 帧头
    
    // <<< MODIFIED: 严格按照 "功能码 -> 长度" 的顺序
    send_buf[i++] = CMD_WAVE_DATA;          // 功能码
    send_buf[i++] = 16;                     // 数据长度 (4个float = 16字节)
    
    // 复制数据
    memcpy(&send_buf[i], &ch1, 4); i += 4;
    memcpy(&send_buf[i], &ch2, 4); i += 4;
    memcpy(&send_buf[i], &ch3, 4); i += 4;
    memcpy(&send_buf[i], &ch4, 4); i += 4;
    
    // <<< MODIFIED: 计算校验和 (从功能码开始，到数据结束)
    for(int j = 1; j < i; j++)
    {
        checksum += send_buf[j];
    }
    send_buf[i++] = checksum; // 填充校验和
    
    HAL_UART_Transmit(debugger_huart, send_buf, i, 100);
}


/* ============================================================================ */
/*                             私有函数实现                                     */
/* ============================================================================ */

static void Debugger_Parse_Data(const uint8_t *data, uint8_t len)
{
    if (data[0] != FRAME_HEADER) return;

    // <<< MODIFIED: 功能码现在是第2个字节 (index 1)
    uint8_t cmd = data[1]; 

    if (cmd == CMD_SET_PID)
    {
        // <<< MODIFIED: 长度现在是第3个字节 (index 2)
        // 检查数据长度是否为12 (3个float)
        if (data[2] == 12) 
        {
            float kp, ki, kd;
            // <<< MODIFIED: 数据现在从第4个字节开始 (index 3)
            memcpy(&kp, &data[3], 4);
            memcpy(&ki, &data[7], 4);
            memcpy(&kd, &data[11], 4);

            MotorA_PID.Speed_PID.Kp = kp; MotorA_PID.Speed_PID.Ki = ki; MotorA_PID.Speed_PID.Kd = kd;
            MotorB_PID.Speed_PID.Kp = kp; MotorB_PID.Speed_PID.Ki = ki; MotorB_PID.Speed_PID.Kd = kd;
            MotorC_PID.Speed_PID.Kp = kp; MotorC_PID.Speed_PID.Ki = ki; MotorC_PID.Speed_PID.Kd = kd;
            MotorD_PID.Speed_PID.Kp = kp; MotorD_PID.Speed_PID.Ki = ki; MotorD_PID.Speed_PID.Kd = kd;
        }
    }
}