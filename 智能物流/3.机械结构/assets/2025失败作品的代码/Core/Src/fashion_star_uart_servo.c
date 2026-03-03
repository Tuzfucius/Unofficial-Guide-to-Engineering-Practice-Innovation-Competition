/*
 * Fashion Star 总线伺服舵机驱动库
 * Version: v0.0.2
 * UpdateTime: 2024/07/17
 */
#include "fashion_star_uart_servo.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// 全局变量，用于同步数据监控
// 注意：这仍然是一个全局变量，但在修复后的函数中被正确地用作了输出目标
extern ServoData servodata[20]; // 假设您要读取20个伺服舵机的数据

// 统一数据包处理

void FSUS_Package2RingBuffer(PackageTypeDef *pkg, RingBufferTypeDef *ringBuf)
{
	uint8_t checksum;
	RingBuffer_WriteUShort(ringBuf, pkg->header);
	RingBuffer_WriteByte(ringBuf, pkg->cmdId);

	// 写入包的长度
	if (pkg->isSync || pkg->size > 255)
	{
		RingBuffer_WriteByte(ringBuf, 0xFF);
		RingBuffer_WriteUShort(ringBuf, pkg->size);
	}
	else
	{
		RingBuffer_WriteByte(ringBuf, (uint8_t)pkg->size);
	}

	RingBuffer_WriteByteArray(ringBuf, pkg->content, pkg->size);
	checksum = RingBuffer_GetChecksum(ringBuf);
	RingBuffer_WriteByte(ringBuf, checksum);
}

// 计算Package的校验和
uint8_t FSUS_CalcChecksum(PackageTypeDef *pkg)
{
	RingBufferTypeDef ringBuf;
	uint8_t pkgBuf[FSUS_PACK_RESPONSE_MAX_SIZE + 7]; // 考虑扩展包头
	RingBuffer_Init(&ringBuf, sizeof(pkgBuf), pkgBuf);
	FSUS_Package2RingBuffer(pkg, &ringBuf);
	return RingBuffer_GetValueByIndex(&ringBuf, RingBuffer_GetByteUsed(&ringBuf) - 1);
}

// 判断是否为有效的请求头的（数据包验证）
FSUS_STATUS FSUS_IsValidResponsePackage(PackageTypeDef *pkg)
{
	if (pkg->header != FSUS_PACK_RESPONSE_HEADER)
		return FSUS_STATUS_WRONG_RESPONSE_HEADER;
	if (pkg->cmdId > FSUS_CMD_NUM)
		return FSUS_STATUS_UNKOWN_CMD_ID;
	if (pkg->size > FSUS_PACK_RESPONSE_MAX_SIZE)
		return FSUS_STATUS_SIZE_TOO_BIG;
	if (FSUS_CalcChecksum(pkg) != pkg->checksum)
		return FSUS_STATUS_CHECKSUM_ERROR;
	return FSUS_STATUS_SUCCESS;
}

// 字节数组转换为数据帧
FSUS_STATUS FSUS_RingBuffer2Package(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg)
{
	// 不再使用malloc，直接填充传入的pkg指针
	if (!pkg)
		return FSUS_STATUS_ERRO;

	// 读取帧头
	pkg->header = RingBuffer_ReadUShort(ringBuf);
	// 读取指令ID
	pkg->cmdId = RingBuffer_ReadByte(ringBuf);
	// 读取包的长度
	uint8_t len_byte = RingBuffer_ReadByte(ringBuf);
	if (len_byte == 0xFF)
	{
		pkg->size = RingBuffer_ReadUShort(ringBuf);
		pkg->isSync = 1;
	}
	else
	{
		pkg->size = len_byte;
		pkg->isSync = 0;
	}
	// 检查size是否超出pkg的content缓冲区大小
	if (pkg->size > sizeof(pkg->content))
	{
		return FSUS_STATUS_SIZE_TOO_BIG;
	}

	RingBuffer_ReadByteArray(ringBuf, pkg->content, pkg->size);
	pkg->checksum = RingBuffer_ReadByte(ringBuf);

	return FSUS_IsValidResponsePackage(pkg);
}
// 发送数据包--------------------------------------------------------
void FSUS_SendPackage_Common(Usart_DataTypeDef *usart, uint8_t cmdId, uint16_t size, uint8_t *content, uint8_t isSync)
{
	PackageTypeDef pkg = {
		.header = FSUS_PACK_REQUEST_HEADER,
		.cmdId = cmdId,
		.size = size,
		.isSync = isSync};
	memcpy(pkg.content, content, size);
	FSUS_Package2RingBuffer(&pkg, usart->sendBuf);
	Usart_SendAll(usart);
}

// 接收数据包（统一处理）--------------------------------------------------------
FSUS_STATUS FSUS_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg)
{
	uint32_t timeout = HAL_GetTick() + FSUS_TIMEOUT_MS;
	uint16_t header_len;

	while (HAL_GetTick() < timeout)
	{
		if (RingBuffer_GetByteUsed(usart->recvBuf) < 4) // 至少需要4字节来判断长度
			continue;

		if (RingBuffer_PeekUShort(usart->recvBuf, 0) != FSUS_PACK_RESPONSE_HEADER)
		{
			RingBuffer_ReadByte(usart->recvBuf); // 丢弃无效字节
			continue;
		}

		uint16_t size;
		if (RingBuffer_PeekByte(usart->recvBuf, 3) == 0xFF)
		{
			if (RingBuffer_GetByteUsed(usart->recvBuf) < 6)
				continue;
			size = RingBuffer_PeekUShort(usart->recvBuf, 4);
			header_len = 6;
		}
		else
		{
			size = RingBuffer_PeekByte(usart->recvBuf, 3);
			header_len = 4;
		}

		uint16_t totalLen = header_len + size + 1; // 包头 + 内容 + 校验和
		if (RingBuffer_GetByteUsed(usart->recvBuf) < totalLen)
			continue;

		// 数据包完整，可以解析
		return FSUS_RingBuffer2Package(usart->recvBuf, pkg);
	}
	return FSUS_STATUS_TIMEOUT;
}

// 同步接收数据包（统一处理）--------------------------------------------------------
FSUS_STATUS FSUS_RecvMonitorPackage_Blocking(Usart_DataTypeDef *usart, PackageTypeDef *pkg)
{
	memset(pkg, 0, sizeof(PackageTypeDef)); // 清空pkg
	uint8_t bIdx = 0;
	uint16_t header_part = 0;
	uint8_t byte;
	uint32_t timeout = HAL_GetTick() + FSUS_TIMEOUT_MS;

	while (HAL_GetTick() < timeout)
	{
		if (RingBuffer_GetByteUsed(usart->recvBuf) == 0)
		{
			continue;
		}
		byte = RingBuffer_ReadByte(usart->recvBuf);

		if (!(pkg->status & FSUS_RECV_FLAG_HEADER))
		{
			if (header_part == 0 && byte == 0x05)
				header_part = byte;
			else if (header_part == 0x05 && byte == 0x1C)
			{
				pkg->header = FSUS_PACK_RESPONSE_HEADER;
				pkg->status |= FSUS_RECV_FLAG_HEADER;
				header_part = 0;
			}
			else
				header_part = 0;
			continue;
		}
		if (!(pkg->status & FSUS_RECV_FLAG_CMD_ID))
		{
			pkg->cmdId = byte;
			if (pkg->cmdId != FSUS_CMD_SET_SERVO_ReadData)
				return FSUS_STATUS_UNKOWN_CMD_ID;
			pkg->status |= FSUS_RECV_FLAG_CMD_ID;
			continue;
		}
		if (!(pkg->status & FSUS_RECV_FLAG_SIZE))
		{
			pkg->size = byte;
			if (pkg->size != 16)
				return FSUS_STATUS_SIZE_TOO_BIG;
			pkg->status |= FSUS_RECV_FLAG_SIZE;
			continue;
		}
		if (!(pkg->status & FSUS_RECV_FLAG_CONTENT))
		{
			pkg->content[bIdx++] = byte;
			if (bIdx >= pkg->size)
				pkg->status |= FSUS_RECV_FLAG_CONTENT;
			continue;
		}
		if (!(pkg->status & FSUS_RECV_FLAG_CHECKSUM))
		{
			pkg->checksum = byte;
			if (FSUS_CalcChecksum(pkg) != pkg->checksum)
				return FSUS_STATUS_CHECKSUM_ERROR;
			return FSUS_STATUS_SUCCESS;
		}
	}
	return FSUS_STATUS_TIMEOUT;
}

// 舵机通讯检测
// 注: 如果没有舵机响应这个Ping指令的话, 就会超时
FSUS_STATUS FSUS_Ping(Usart_DataTypeDef *usart, uint8_t servo_id)
{
	FSUS_STATUS statusCode;
	uint8_t ehcoServoId;
	FSUS_SendPackage_Common(usart, FSUS_CMD_PING, 1, &servo_id, 0);

	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		if (pkg.cmdId != FSUS_CMD_PING)
			return FSUS_STATUS_WRONG_RESPONSE_CMD;
		ehcoServoId = (uint8_t)pkg.content[0];
		if (ehcoServoId != servo_id)
			return FSUS_STATUS_ID_NOT_MATCH;
	}
	return statusCode;
}

// 重置舵机的用户资料
FSUS_STATUS FSUS_ResetUserData(Usart_DataTypeDef *usart, uint8_t servo_id)
{
	FSUS_STATUS statusCode;
	FSUS_SendPackage_Common(usart, FSUS_CMD_RESET_USER_DATA, 1, &servo_id, 0);

	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		if (pkg.cmdId != FSUS_CMD_RESET_USER_DATA)
			return FSUS_STATUS_WRONG_RESPONSE_CMD;
		uint8_t result = (uint8_t)pkg.content[1];
		return (result == 1) ? FSUS_STATUS_SUCCESS : FSUS_STATUS_FAIL;
	}
	return statusCode;
}

// 读取数据
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t *size)
{
	FSUS_STATUS statusCode;
	// 构造content
	uint8_t buffer[2] = {servo_id, address};
	// 发送请求数据
	FSUS_SendPackage_Common(usart, FSUS_CMD_READ_DATA, 2, buffer, 0);
	// 接收返回信息
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		// 读取数据
		// 读取数据是多少个位
		*size = pkg.size - 2; // content的长度减去servo_id跟address的长度
		// 数据拷贝
		for (int i = 0; i < *size; i++)
		{
			value[i] = pkg.content[i + 2];
		}
	}
	return statusCode;
}

// 写入数据
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size)
{
	FSUS_STATUS statusCode;
#define FSUS_WRITE_MAX_SIZE 32
	uint8_t buffer[FSUS_WRITE_MAX_SIZE]; // 固定最大长度
	int i;
	// 检查 size+2 是否越界
	if ((size + 2) > FSUS_WRITE_MAX_SIZE)
		return FSUS_STATUS_SIZE_TOO_BIG;
	buffer[0] = servo_id;
	buffer[1] = address;
	// 拷贝数据
	for (i = 0; i < size; i++)
	{
		buffer[i + 2] = value[i];
	}
	// 发送请求数据
	FSUS_SendPackage_Common(usart, FSUS_CMD_WRITE_DATA, size + 2, buffer, 0);
	// 接收返回信息
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		uint8_t result = pkg.content[2];
		if (result == 1)
		{
			statusCode = FSUS_STATUS_SUCCESS;
		}
		else
		{
			statusCode = FSUS_STATUS_FAIL;
		}
	}
	return statusCode;
}

// 设置舵机的角度
// @angle 单位度
// @interval 单位ms
// @power 舵机执行功率 单位mW
//        若power=0或者大于保护值
FSUS_STATUS FSUS_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t power)
{
	// 创建环形缓冲队列
	const uint8_t size = 7;
	uint8_t buffer[size + 1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	// 数值约束
	if (angle > 180.0f)
	{
		angle = 180.0f;
	}
	else if (angle < -180.0f)
	{
		angle = -180.0f;
	}
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, power);
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage_Common(usart, FSUS_CMD_ROTATE, size, buffer + 1, 0);

	return FSUS_STATUS_SUCCESS;
}

/* 设置舵机的角度(指定周期) */
FSUS_STATUS FSUS_SetServoAngleByInterval(Usart_DataTypeDef *usart, uint8_t servo_id,
										 float angle, uint16_t interval, uint16_t t_acc,
										 uint16_t t_dec, uint16_t power)
{
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size + 1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	// 数值约束
	if (angle > 180.0f)
	{
		angle = 180.0f;
	}
	else if (angle < -180.0f)
	{
		angle = -180.0f;
	}
	if (t_acc < 20)
	{
		t_acc = 20;
	}
	if (t_dec < 20)
	{
		t_dec = 20;
	}

	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);

	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL, size, buffer + 1, 0);

	return FSUS_STATUS_SUCCESS;
}

/* 设置舵机的角度(指定转速) */
FSUS_STATUS FSUS_SetServoAngleByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id,
										 float angle, float velocity, uint16_t t_acc,
										 uint16_t t_dec, uint16_t power)
{
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size + 1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);

	// 数值约束
	if (angle > 180.0f)
	{
		angle = 180.0f;
	}
	else if (angle < -180.0f)
	{
		angle = -180.0f;
	}
	if (velocity < 1.0f)
	{
		velocity = 1.0f;
	}
	else if (velocity > 750.0f)
	{
		velocity = 750.0f;
	}
	if (t_acc < 20)
	{
		t_acc = 20;
	}
	if (t_dec < 20)
	{
		t_dec = 20;
	}

	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10.0f * angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f * velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);

	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY, size, buffer + 1, 0);

	return FSUS_STATUS_SUCCESS;
}

/* 查询单个舵机的角度信息 angle 单位度 */
FSUS_STATUS FSUS_QueryServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle)
{
	const uint8_t size = 1; // 请求包content的长度
	uint8_t ehcoServoId;
	int16_t echoAngle;

	// 发送舵机角度请求包
	FSUS_SendPackage_Common(usart, FSUS_CMD_READ_ANGLE, size, &servo_id, 0);
	// 接收返回的Ping
	PackageTypeDef pkg;
	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		// 成功的获取到舵机角度回读数据
		ehcoServoId = (uint8_t)pkg.content[0];
		// 检测舵机ID是否匹配
		if (ehcoServoId != servo_id)
		{
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}

		// 提取舵机角度
		echoAngle = (int16_t)(pkg.content[1] | (pkg.content[2] << 8));
		*angle = (float)(echoAngle / 10.0);
	}
	return statusCode;
}

// /* 设置舵机的角度(多圈模式) */
// FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
// 									uint32_t interval, uint16_t power)
// {
// 	// 创建环形缓冲队列
// 	const uint8_t size = 11;
// 	uint8_t buffer[size + 1];
// 	RingBufferTypeDef ringBuf;
// 	RingBuffer_Init(&ringBuf, size, buffer);
// 	// 数值约束
// 	if (angle > 368640.0f)
// 	{
// 		angle = 368640.0f;
// 	}
// 	else if (angle < -368640.0f)
// 	{
// 		angle = -368640.0f;
// 	}
// 	if (interval > 4096000)
// 	{
// 		angle = 4096000;
// 	}
// 	// 协议打包
// 	RingBuffer_WriteByte(&ringBuf, servo_id);
// 	RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * angle));
// 	RingBuffer_WriteULong(&ringBuf, interval);
// 	RingBuffer_WriteShort(&ringBuf, power);

// 	// 发送请求包
// 	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
// 	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN, size, buffer + 1, 0);

// 	return FSUS_STATUS_SUCCESS;
// }
FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
                                    uint32_t interval, uint16_t power)
{
    // 1. 定义 content 数组的长度
    const uint8_t content_size = 11;

    // 2. 直接在栈上创建 content 数组
    uint8_t content[content_size];

    // 3. 数值约束
    if (angle > 368640.0f) angle = 368640.0f;
    else if (angle < -368640.0f) angle = -368640.0f;

    // 【重要修正】修复逻辑错误
    if (interval > 4096000)
    {
        interval = 4096000; // 应该是 interval 被赋值
    }

    // 4. 直接构建 content 数组
    int32_t angle_int = (int32_t)(angle * 10.0f);

    content[0] = servo_id;

    // 手动进行小端模式打包 (4字节)
    content[1] = (uint8_t)(angle_int & 0xFF);
    content[2] = (uint8_t)((angle_int >> 8) & 0xFF);
    content[3] = (uint8_t)((angle_int >> 16) & 0xFF);
    content[4] = (uint8_t)((angle_int >> 24) & 0xFF);

    // 手动进行小端模式打包 (4字节)
    content[5] = (uint8_t)(interval & 0xFF);
    content[6] = (uint8_t)((interval >> 8) & 0xFF);
    content[7] = (uint8_t)((interval >> 16) & 0xFF);
    content[8] = (uint8_t)((interval >> 24) & 0xFF);

    // 手动进行小端模式打包 (2字节)
    content[9] = (uint8_t)(power & 0xFF);
    content[10] = (uint8_t)((power >> 8) & 0xFF);

    // 5. 发送请求包
    FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN, content_size, content, 0);

    return FSUS_STATUS_SUCCESS;
}

// /* 设置舵机的角度(多圈模式, 指定周期) */
// FSUS_STATUS FSUS_SetServoAngleMTurnByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
// 											  uint32_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power)
// {
// 	// 创建环形缓冲队列
// 	const uint8_t size = 15;
// 	uint8_t buffer[size + 1];
// 	RingBufferTypeDef ringBuf;
// 	RingBuffer_Init(&ringBuf, size, buffer);

// 	// 数值约束
// 	if (angle > 368640.0f)
// 	{
// 		angle = 368640.0f;
// 	}
// 	else if (angle < -368640.0f)
// 	{
// 		angle = -368640.0f;
// 	}
// 	if (interval > 4096000)
// 	{
// 		interval = 4096000;
// 	}
// 	if (t_acc < 20)
// 	{
// 		t_acc = 20;
// 	}
// 	if (t_dec < 20)
// 	{
// 		t_dec = 20;
// 	}
// 	// 协议打包
// 	RingBuffer_WriteByte(&ringBuf, servo_id);
// 	RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * angle));
// 	RingBuffer_WriteULong(&ringBuf, interval);
// 	RingBuffer_WriteUShort(&ringBuf, t_acc);
// 	RingBuffer_WriteUShort(&ringBuf, t_dec);
// 	RingBuffer_WriteShort(&ringBuf, power);

// 	// 发送请求包
// 	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
// 	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL, size, buffer + 1, 0);

// 	return FSUS_STATUS_SUCCESS;
// }

/* 设置舵机的角度(多圈模式, 指定转速) */
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle,
											  float velocity, uint16_t t_acc, uint16_t t_dec, uint16_t power)
{
	// 创建环形缓冲队列
	const uint8_t size = 13;
	uint8_t buffer[size + 1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	// 数值约束
	if (angle > 368640.0f)
	{
		angle = 368640.0f;
	}
	else if (angle < -368640.0f)
	{
		angle = -368640.0f;
	}
	if (velocity < 1.0f)
	{
		velocity = 1.0f;
	}
	else if (velocity > 750.0f)
	{
		velocity = 750.0f;
	}
	if (t_acc < 20)
	{
		t_acc = 20;
	}
	if (t_dec < 20)
	{
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10.0f * angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f * velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);

	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY, size, buffer + 1, 0);

	return FSUS_STATUS_SUCCESS;
}

/* 查询舵机的角度(多圈模式) */
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle)
{
	// 创建环形缓冲队列
	const uint8_t size = 1; // 请求包content的长度
	uint8_t ehcoServoId;
	int32_t echoAngle;

	// 发送舵机角度请求包
	FSUS_SendPackage_Common(usart, FSUS_CMD_QUERY_SERVO_ANGLE_MTURN, size, &servo_id, 0);
	// 接收返回的Ping
	PackageTypeDef pkg;
	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		// 成功的获取到舵机角度回读数据
		ehcoServoId = (uint8_t)pkg.content[0];
		// 检测舵机ID是否匹配
		if (ehcoServoId != servo_id)
		{
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}

		// 提取舵机角度
		// echoAngle = (int32_t)(pkg.content[1] | (pkg.content[2] << 8) | (pkg.content[3] << 16) | (pkg.content[4] << 24));
		echoAngle = *((int32_t *)&pkg.content[1]);
		*angle = (float)echoAngle / 10.0f;
		// *angle = (float)(echoAngle / 10.0);
	}
	return statusCode;
}

/* 舵机阻尼模式 */
FSUS_STATUS FSUS_DampingMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint16_t power)
{
	const uint8_t size = 3;					 // 请求包content的长度
	uint8_t buffer[size + 1];				 // content缓冲区
	RingBufferTypeDef ringBuf;				 // 创建环形缓冲队列
	RingBuffer_Init(&ringBuf, size, buffer); // 缓冲队列初始化
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteUShort(&ringBuf, power);
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage_Common(usart, FSUS_CMD_DAMPING, size, buffer + 1, 0);
	return FSUS_STATUS_SUCCESS;
}

// 舵机重置多圈角度圈数
FSUS_STATUS FSUS_ServoAngleReset(Usart_DataTypeDef *usart, uint8_t servo_id)
{
	FSUS_STATUS statusCode;
	FSUS_SendPackage_Common(usart, FSUS_CMD_RESERT_SERVO_ANGLE_MTURN, 1, &servo_id, 0);

	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		if (pkg.cmdId != FSUS_CMD_RESERT_SERVO_ANGLE_MTURN)
			return FSUS_STATUS_WRONG_RESPONSE_CMD;
		uint8_t result = (uint8_t)pkg.content[1];
		return (result == 1) ? FSUS_STATUS_SUCCESS : FSUS_STATUS_FAIL;
	}
	return statusCode;
}
/*零点设置 仅适用于无刷磁编码舵机*/
FSUS_STATUS FSUS_SetOriginPoint(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t reset)
{
	FSUS_STATUS statusCode;
	uint8_t content[2] = {servo_id, reset};
	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_ORIGIN_POINT, 2, content, 0);

	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS)
	{
		if (pkg.cmdId != FSUS_CMD_SET_ORIGIN_POINT)
			return FSUS_STATUS_WRONG_RESPONSE_CMD;
		uint8_t result = (uint8_t)pkg.content[1];
		return (result == 1) ? FSUS_STATUS_SUCCESS : FSUS_STATUS_FAIL;
	}
	return statusCode;
}

/* 舵机开始异步命令*/
FSUS_STATUS FSUS_BeginAsync(Usart_DataTypeDef *usart)
{
	// 根据协议，请求包内容长度为0，所以content为NULL，size为0
	FSUS_SendPackage_Common(usart, FSUS_CMD_BEGIN_ASYNC, 0, NULL, 0);
	// 此命令无回应
	return FSUS_STATUS_SUCCESS;
}

/* 舵机结束异步命令*/
FSUS_STATUS FSUS_EndAsync(Usart_DataTypeDef *usart, uint8_t cancel)
{
	FSUS_SendPackage_Common(usart, FSUS_CMD_END_ASYNC, 1, &cancel, 0);
	// 此命令无回应
	return FSUS_STATUS_SUCCESS;
}

/* 舵机单个数据监控*/
FSUS_STATUS FSUS_ServoMonitor(Usart_DataTypeDef *usart, uint8_t servo_id, ServoData servodata[])
{

	// 创建环形缓冲队列
	const uint8_t size = 1;
	uint8_t buffer[size + 1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	double temp; // 温度数据转换
	RingBuffer_WriteByte(&ringBuf, servo_id);
	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_ReadData, (uint8_t)size, buffer + 1, 0);

	PackageTypeDef pkg;
	FSUS_STATUS status = FSUS_RecvPackage(usart, &pkg); // FSUS状态码

	if (status != FSUS_STATUS_SUCCESS)
	{
		return status; // 如果接收失败，返回错误状态
	}

	// 解析当前数据包内容
	servodata[0].id = pkg.content[0];
	servodata[0].voltage = (int16_t)((pkg.content[2] << 8) | pkg.content[1]);
	servodata[0].current = (int16_t)((pkg.content[4] << 8) | pkg.content[3]);
	servodata[0].power = (int16_t)((pkg.content[6] << 8) | pkg.content[5]);
	servodata[0].temperature = (int16_t)((pkg.content[8] << 8) | pkg.content[7]);
	temp = (double)servodata[0].temperature;
	servodata[0].temperature = 1 / (log(temp / (4096.0f - temp)) / 3435.0f + 1 / (273.15 + 25)) - 273.15;
	servodata[0].status = pkg.content[9];
	// servodata[0].angle = (int32_t)((pkg.content[13] << 24) | (pkg.content[12] << 16) | (pkg.content[11] << 8) | pkg.content[10]);
	// servodata[0].angle = (float)(servodata[0].angle / 10.0f);
	int32_t raw_angle = *((int32_t *)&pkg.content[10]);
	servodata[0].angle = (float)raw_angle / 10.0f;
	servodata[0].circle_count = (int16_t)((pkg.content[15] << 8) | pkg.content[14]);

	// 重置 pkg 状态以接收下一组数据
	pkg.status = 0;

	return FSUS_STATUS_SUCCESS;
}

/* 舵机控制模式停止指令*/
// mode 指令停止形式
// 0-停止后卸力(失锁)
// 1-停止后保持锁力
// 2-停止后进入阻尼状态
FSUS_STATUS FSUS_StopOnControlMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t mode, uint16_t power)
{
	// 创建环形缓冲队列
	const uint8_t size = 4;
	uint8_t buffer[size + 1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);

	RingBuffer_WriteByte(&ringBuf, servo_id);
	mode = mode | 0x10;
	RingBuffer_WriteByte(&ringBuf, mode);
	RingBuffer_WriteShort(&ringBuf, power);

	FSUS_SendPackage_Common(usart, FSUS_CMD_CONTROL_MODE_STOP, (uint8_t)size, buffer + 1, 0);

	return FSUS_STATUS_SUCCESS;
}

/**
 * @brief 同步命令控制函数 (已修复和优化)
 * @note  这是所有同步运动的核心。它构建一个大的数据包，其中包含了针对多个舵机的多个子指令。
 * @param usart         串口实例指针
 * @param servo_count   要同步控制的舵机数量
 * @param ServoMode     要执行的同步模式 (参考 fashion_star_uart_servo.h 中的宏定义)
 * @param servoSync     包含所有舵机目标参数的结构体数组
 * @param servo_data_out 如果模式是数据监控(MODE_Query_SERVO_Monitor), 此数组用于接收返回的数据。其他模式下此参数被忽略。
 * @return              FSUS_STATUS 操作的状态码
 */
FSUS_STATUS FSUS_SyncCommand(Usart_DataTypeDef *usart, uint8_t servo_count, uint8_t ServoMode, FSUS_sync_servo servoSync[], ServoData servo_data_out[])
{
	// 缓冲区需要足够大以容纳最大的可能指令 (多圈带加减速)
	uint8_t buffer[3 + servo_count * 15];
	uint16_t size = 0;
	RingBufferTypeDef ringBuf;
	uint8_t cmd_id = 0;
	uint8_t content_len_per_servo = 0;

	// 根据不同的同步模式，设置子指令ID和每个舵机的数据长度
	switch (ServoMode)
	{
	case MODE_SET_SERVO_ANGLE:
		cmd_id = FSUS_CMD_ROTATE;
		content_len_per_servo = 7;
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		// 写入同步命令的头部: [子指令ID][单条指令长度][舵机数量]
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		// 依次写入每个舵机的数据
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * servoSync[i].angle));
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].interval_single);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].power);
		}
		break;

	case MODE_SET_SERVO_ANGLE_BY_INTERVAL:
		cmd_id = FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL;
		content_len_per_servo = 11;
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * servoSync[i].angle));
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].interval_single);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].power);
		}
		break;

	case MODE_SET_SERVO_ANGLE_BY_VELOCITY:
		cmd_id = FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY;
		content_len_per_servo = 11;
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			RingBuffer_WriteShort(&ringBuf, (int16_t)(10 * servoSync[i].angle));
			RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f * servoSync[i].velocity));
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].power);
		}
		break;

	case MODE_SET_SERVO_ANGLE_MTURN:
		cmd_id = FSUS_CMD_SET_SERVO_ANGLE_MTURN;
		content_len_per_servo = 11;
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * servoSync[i].angle));
			RingBuffer_WriteULong(&ringBuf, servoSync[i].interval_multi);
			RingBuffer_WriteShort(&ringBuf, servoSync[i].power);
		}
		break;

	case MODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL:
		cmd_id = FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL;
		content_len_per_servo = 15;
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * servoSync[i].angle));
			RingBuffer_WriteULong(&ringBuf, servoSync[i].interval_multi);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
			RingBuffer_WriteShort(&ringBuf, servoSync[i].power);
		}
		break;

	case MODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY:
		cmd_id = FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY;
		content_len_per_servo = 13;
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			RingBuffer_WriteLong(&ringBuf, (int32_t)(10 * servoSync[i].angle));
			RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f * servoSync[i].velocity));
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_acc);
			RingBuffer_WriteUShort(&ringBuf, servoSync[i].t_dec);
			RingBuffer_WriteShort(&ringBuf, servoSync[i].power);
		}
		break;

	case MODE_Query_SERVO_Monitor:
		cmd_id = FSUS_CMD_SET_SERVO_ReadData;
		content_len_per_servo = 1; // 每个舵机只需要1字节ID
		size = 3 + servo_count * content_len_per_servo;
		RingBuffer_Init(&ringBuf, size, buffer);
		RingBuffer_WriteByte(&ringBuf, cmd_id);
		RingBuffer_WriteByte(&ringBuf, content_len_per_servo);
		RingBuffer_WriteByte(&ringBuf, servo_count);
		// 在调用接收函数前，需要将输出数组中的ID预先填好，以便接收函数进行匹配
		for (int i = 0; i < servo_count; i++)
		{
			RingBuffer_WriteByte(&ringBuf, servoSync[i].id);
			// 如果调用者提供了输出数组，就预设ID
			if (servo_data_out != NULL)
			{
				servo_data_out[i].id = servoSync[i].id;
			}
		}
		break;

	default:
		// 如果传入了无效的模式，返回错误
		return FSUS_STATUS_INVALID_PARAM;
	}

	// 检查size是否有效，防止后续操作错误
	if (size == 0)
	{
		return FSUS_STATUS_ERRO;
	}

	// 调用底层函数发送最终构建好的数据包
	// 第一个参数是主指令ID，固定为同步命令ID
	// 最后一个参数 `isSync` 用于判断是否需要使用扩展包头 (数据长度超过255)
	FSUS_SendPackage_Common(usart, FSUS_CMD_SET_SERVO_SyncCommand, size, buffer + 1, (size > 255));

	// 只有数据监控模式需要等待并接收舵机的回应
	if (ServoMode == MODE_Query_SERVO_Monitor)
	{
		// 检查调用者是否传入了有效的数组来接收数据
		if (servo_data_out == NULL)
		{
			return FSUS_STATUS_INVALID_PARAM;
		}
		// 调用专门的接收函数来处理多个舵机的回应包
		return FSUS_SyncServoMonitor(usart, servo_count, servo_data_out);
	}

	// 其他所有设置类同步命令，根据协议是不回应的，因此直接返回成功
	return FSUS_STATUS_SUCCESS;
}

/**
 * @brief 接收并解析多个舵机的数据监控回应包 (同步查询的辅助函数)
 * @note  此函数会循环等待，直到接收到指定数量(servo_count)的回应包，或等待超时。
 *        它依赖于一个专门的、阻塞式的接收函数 `FSUS_RecvMonitorPackage_Blocking`。
 * @param usart           串口实例指针
 * @param servo_count     期望接收的回应包数量 (等于查询的舵机数量)
 * @param servo_data_out  用于存储解析后数据的外部数组。此数组的ID必须在调用前被预设好。
 * @return                FSUS_STATUS 操作的状态码
 */
FSUS_STATUS FSUS_SyncServoMonitor(Usart_DataTypeDef *usart, uint8_t servo_count, ServoData servo_data_out[])
{
	PackageTypeDef pkg;
	FSUS_STATUS status;
	double temp_adc;		  // 用于温度计算的临时变量
	uint8_t packet_count = 0; // 已成功接收并解析的数据包计数

	// 循环接收数据包，直到收满 servo_count 个或者超时
	while (packet_count < servo_count)
	{
		// 调用专门的、阻塞式的函数来接收一个格式固定的数据监控包
		status = FSUS_RecvMonitorPackage_Blocking(usart, &pkg);

		// 如果接收失败 (包括超时)，则立即中断并返回错误状态
		if (status != FSUS_STATUS_SUCCESS)
		{
			return status;
		}

		// 解析当前接收到的数据包内容
		uint8_t current_id = pkg.content[0];

		// 在输出数组中查找与当前回应ID匹配的舵机，并填充数据
		// 这种设计允许舵机的回应顺序是任意的
		int found = 0;
		for (int i = 0; i < servo_count; i++)
		{
			// 检查调用者是否已预设好ID
			if (servo_data_out[i].id == current_id)
			{
				servo_data_out[i].voltage = (int16_t)((pkg.content[2] << 8) | pkg.content[1]);
				servo_data_out[i].current = (int16_t)((pkg.content[4] << 8) | pkg.content[3]);
				servo_data_out[i].power = (int16_t)((pkg.content[6] << 8) | pkg.content[5]);

				temp_adc = (double)((int16_t)((pkg.content[8] << 8) | pkg.content[7]));
				// Steinhart-Hart 方程反算温度
				servo_data_out[i].temperature = 1.0f / (logf(temp_adc / (4096.0f - temp_adc)) / 3435.0f + 1.0f / (273.15f + 25.0f)) - 273.15f;

				servo_data_out[i].status = pkg.content[9];

				int32_t raw_angle = (int32_t)((pkg.content[13] << 24) | (pkg.content[12] << 16) | (pkg.content[11] << 8) | pkg.content[10]);
				servo_data_out[i].angle = (float)(raw_angle / 10.0f);

				servo_data_out[i].circle_count = (int16_t)((pkg.content[15] << 8) | pkg.content[14]);

				found = 1;
				break; // 找到并填充后，退出内层循环
			}
		}

		// 如果找到了匹配的ID并填充了数据，则成功接收的包数量加一
		if (found)
		{
			packet_count++;
		}
		// 如果在输出数组中没有找到匹配的ID，可以忽略这个包，或者返回一个错误
		// 当前选择忽略，继续等待下一个包
	}

	// 清空接收缓冲区中可能残留的任何数据，为下一次通信做准备
	RingBuffer_Reset(usart->recvBuf);

	// 如果成功接收了所有预期的数据包，返回成功
	return FSUS_STATUS_SUCCESS;
}