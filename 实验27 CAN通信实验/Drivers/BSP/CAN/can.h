/**
 ****************************************************************************************************
 * @file        can.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2023-06-06
 * @brief       CAN 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211025
 * 第一次发布
 * V1.1 20230606
 * 1, 优化can_send_msg函数, 新增发送超时处理机制
 ****************************************************************************************************
 */

#ifndef __CAN_H
#define __CAN_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* CAN 引脚 定义 */

#define CAN_RX_GPIO_PORT                GPIOA
#define CAN_RX_GPIO_PIN                 GPIO_PIN_11
#define CAN_RX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define CAN_TX_GPIO_PORT                GPIOA
#define CAN_TX_GPIO_PIN                 GPIO_PIN_12
#define CAN_TX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

/******************************************************************************************/

/* CAN接收RX0中断使能: 1, 中断接收(报文入软件队列, 推荐); 0, 主循环轮询接收 */
#define CAN_RX0_INT_ENABLE              1

/* 软件接收队列深度, 必须是2的幂, 队列满时丢弃最新报文 */
#define CAN_RX_QUEUE_DEPTH              32

/* 发送完成等待超时(ms), 总线上没有其他节点应答时会超时 */
#define CAN_TX_TIMEOUT_MS               10

/******************************************************************************************/
/* 一帧CAN报文(标准帧/扩展帧通用), 应用层只和这个结构体打交道 */
typedef struct
{
    uint32_t id;        /* 标准ID(11位) 或 扩展ID(29位) */
    uint8_t  ide;       /* CAN_ID_STD / CAN_ID_EXT */
    uint8_t  rtr;       /* CAN_RTR_DATA / CAN_RTR_REMOTE */
    uint8_t  dlc;       /* 数据长度 0~8 */
    uint8_t  data[8];   /* 数据 */
} can_msg_t;

/* 函数声明 */
uint8_t can_init(uint32_t tsjw,uint32_t tbs2,uint32_t tbs1,uint16_t brp,uint32_t mode); /* CAN初始化 */
uint8_t can_filter_config(uint32_t id, uint32_t mask);                                   /* 标准帧过滤器配置, mask中为1的位必须与id一致 */
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len);                           /* CAN发送数据: 标准帧+数据帧 */
uint8_t can_send_msg_ex(uint32_t id, uint32_t ide, uint32_t rtr, uint8_t *msg, uint8_t len); /* CAN发送数据: 可选标准/扩展/远程帧 */
uint8_t can_recv_msg(can_msg_t *msg);                                                   /* 从软件队列取一帧, 0:无数据; 1:取到 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf);                                     /* 兼容旧接口: 按ID取一帧(会丢弃不匹配的报文) */
uint32_t can_get_error(void);                                                           /* 读取CAN错误码, 0表示正常 */

#endif








