/**
 ****************************************************************************************************
 * @file        can.c
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

#include "./BSP/CAN/can.h"
#include "./BSP/LED/led.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"


CAN_HandleTypeDef   g_canx_handler;     /* CANx句柄 */
CAN_TxHeaderTypeDef g_canx_txheader;    /* 发送参数句柄 */
CAN_RxHeaderTypeDef g_canx_rxheader;    /* 接收参数句柄 */

/*
 * 软件接收队列。
 * 中断里只做“把硬件FIFO里的报文搬到队列”, 主循环再慢慢处理,
 * 这样中断耗时恒定, 也不会因为应用层处理慢而丢报文。
 * head 只在中断侧修改, tail 只在主循环侧修改, 单生产者/单消费者无需关中断。
 */
static can_msg_t g_can_rx_queue[CAN_RX_QUEUE_DEPTH];
static volatile uint16_t g_can_rx_head = 0;     /* 写指针 */
static volatile uint16_t g_can_rx_tail = 0;     /* 读指针 */
static uint8_t g_can_dummy_data[8];             /* 远程帧/零长帧发送时的占位数据 */

/**
 * @brief       CAN初始化
 * @param       tsjw    : 重新同步跳跃时间单元.范围: 1~3;
 * @param       tbs2    : 时间段2的时间单元.范围: 1~8;
 * @param       tbs1    : 时间段1的时间单元.范围: 1~16;
 * @param       brp     : 波特率分频器.范围: 1~1024;
 *   @note      以上4个参数, 在函数内部会减1, 所以, 任何一个参数都不能等于0
 *              CAN挂在APB1上面, 其输入时钟频率为 Fpclk1 = PCLK1 = 42Mhz
 *              tq     = brp * tpclk1;
 *              波特率 = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *              例如 can_init(CAN_SJW_1TQ, CAN_BS2_3TQ, CAN_BS1_10TQ, 6, CAN_MODE_NORMAL):
 *              42M / ((10 + 3 + 1) * 6) = 500Kbps, 采样点 78.6%
 *
 * @param       mode    : CAN_MODE_NORMAL,  普通模式;
                          CAN_MODE_LOOPBACK,回环模式;
 * @retval      0,  初始化成功; 其他, 初始化失败;
 */
uint8_t can_init(uint32_t tsjw, uint32_t tbs2, uint32_t tbs1, uint16_t brp, uint32_t mode)
{
    g_canx_handler.Instance = CAN1;
    g_canx_handler.Init.Prescaler = brp;                /* 分频系数(Fdiv)为brp+1 */
    g_canx_handler.Init.Mode = mode;                    /* 模式设置 */
    g_canx_handler.Init.SyncJumpWidth = tsjw;           /* 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ */
    g_canx_handler.Init.TimeSeg1 = tbs1;                /* tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ */
    g_canx_handler.Init.TimeSeg2 = tbs2;                /* tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ */
    g_canx_handler.Init.TimeTriggeredMode = DISABLE;    /* 非时间触发通信模式 */
    g_canx_handler.Init.AutoBusOff = DISABLE;           /* 软件自动离线管理 */
    g_canx_handler.Init.AutoWakeUp = DISABLE;           /* 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位) */
    g_canx_handler.Init.AutoRetransmission = ENABLE;    /* 使能报文自动重传(总线无应答时会在超时后中止) */
    g_canx_handler.Init.ReceiveFifoLocked = DISABLE;    /* 报文不锁定,新的覆盖旧的 */
    g_canx_handler.Init.TransmitFifoPriority = DISABLE; /* 优先级由报文标识符决定 */
    if (HAL_CAN_Init(&g_canx_handler) != HAL_OK)
    {
        return 1;
    }

    /* 默认全通过(id=0, mask=0), 应用层再按ID分发; 需要时用 can_filter_config() 收紧 */
    if (can_filter_config(0x000, 0x000) != 0)
    {
        return 2;
    }

    /* 启动CAN外围设备 */
    if (HAL_CAN_Start(&g_canx_handler) != HAL_OK)
    {
        return 3;
    }

#if CAN_RX0_INT_ENABLE

    /* 使用中断接收(必须在HAL_CAN_Start之后使能) */
    __HAL_CAN_ENABLE_IT(&g_canx_handler, CAN_IT_RX_FIFO0_MSG_PENDING); /* FIFO0消息挂号中断允许 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);                         /* 抢占优先级1，子优先级0 */
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);                                 /* 使能CAN中断 */
#endif

    return 0;
}

/**
 * @brief       CAN过滤器配置(标准帧, 32位掩码模式)
 * @param       id      : 需要匹配的标准ID(11位), 按位与mask比较
 * @param       mask    : 掩码, 为1的位必须和id一致; mask=0 表示全部接收
 * @retval      0, 成功; 1, 失败
 *   @note      CANopen中从机只关心自己的报文时, 可这样配置:
 *              接收 0x600+NodeId : can_filter_config(0x600 + NodeId, 0x7FF);
 *              (过滤器bank0被本函数占用, 需要多组过滤器时请自行扩展bank号)
 */
uint8_t can_filter_config(uint32_t id, uint32_t mask)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                             /* 过滤器0 */
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;         /* 掩码模式 */
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;        /* 32位 */
    sFilterConfig.FilterIdHigh = (uint16_t)((id & 0x7FF) << 5);        /* 标准ID放在高16位, 左移5位 */
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (uint16_t)((mask & 0x7FF) << 5);  /* 标准ID掩码 */
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;    /* 过滤器0关联到FIFO0 */
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;       /* 激活过滤器 */
    sFilterConfig.SlaveStartFilterBank = 14;

    return (HAL_CAN_ConfigFilter(&g_canx_handler, &sFilterConfig) == HAL_OK) ? 0 : 1;
}

/**
 * @brief       CAN底层驱动，引脚配置，时钟配置，中断配置
                此函数会被HAL_CAN_Init()调用
 * @param       hcan:CAN句柄
 * @retval      无
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    if (CAN1 == hcan->Instance)
    {
        CAN_RX_GPIO_CLK_ENABLE();       /* CAN_RX脚时钟使能 */
        CAN_TX_GPIO_CLK_ENABLE();       /* CAN_TX脚时钟使能 */
        __HAL_RCC_CAN1_CLK_ENABLE();    /* 使能CAN1时钟 */

        GPIO_InitTypeDef gpio_init_struct;

        gpio_init_struct.Pin = CAN_TX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(CAN_TX_GPIO_PORT, &gpio_init_struct); /* CAN_TX脚 模式设置 */

        gpio_init_struct.Pin = CAN_RX_GPIO_PIN;
        HAL_GPIO_Init(CAN_RX_GPIO_PORT, &gpio_init_struct); /* CAN_RX脚 必须设置成输入模式 */
    }
}

/**
 * @brief       把硬件RX FIFO0中的报文全部搬进软件队列
 *   @note      中断模式下由CAN1_RX0_IRQHandler调用; 轮询模式下由can_recv_msg调用
 * @param       无
 * @retval      无
 */
static void can_rx_fifo_to_queue(void)
{
    can_msg_t msg;
    CAN_RxHeaderTypeDef rxheader;
    uint16_t next;

    while (HAL_CAN_GetRxFifoFillLevel(&g_canx_handler, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&g_canx_handler, CAN_RX_FIFO0, &rxheader, msg.data) != HAL_OK)
        {
            break;
        }

        msg.ide = rxheader.IDE;
        msg.rtr = rxheader.RTR;
        msg.dlc = (rxheader.DLC > 8) ? 8 : rxheader.DLC;
        msg.id  = (rxheader.IDE == CAN_ID_STD) ? rxheader.StdId : rxheader.ExtId;

        next = (uint16_t)((g_can_rx_head + 1) & (CAN_RX_QUEUE_DEPTH - 1));

        if (next != g_can_rx_tail)      /* 队列未满才入队, 满了丢弃最新报文 */
        {
            g_can_rx_queue[g_can_rx_head] = msg;
            g_can_rx_head = next;
        }
    }
}

#if CAN_RX0_INT_ENABLE /* 使能RX0中断 */

/**
 * @brief       CAN RX0 中断服务函数
 *   @note      交给HAL统一清标志, 再通过回调把报文搬进软件队列
 * @param       无
 * @retval      无
 */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&g_canx_handler);
}

/**
 * @brief       FIFO0收到报文回调(由HAL_CAN_IRQHandler调用)
 * @param       hcan: CAN句柄
 * @retval      无
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        can_rx_fifo_to_queue();
    }
}

#endif

/**
 * @brief       CAN 发送一组数据
 *   @note      发送格式固定为: 标准ID, 数据帧
 * @param       id      : 标准ID(11位)
 * @param       msg     : 数据指针
 * @param       len     : 数据长度
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    return can_send_msg_ex(id, CAN_ID_STD, CAN_RTR_DATA, msg, len);
}

/**
 * @brief       CAN 发送数据(支持标准/扩展/远程帧)
 * @param       id      : 标准ID(11位) 或 扩展ID(29位)
 * @param       ide     : CAN_ID_STD / CAN_ID_EXT
 * @param       rtr     : CAN_RTR_DATA / CAN_RTR_REMOTE
 * @param       msg     : 数据指针, 远程帧可为NULL
 * @param       len     : 数据长度 0~8
 * @retval      发送状态 0, 成功; 1, 失败(邮箱满或无应答超时)
 */
uint8_t can_send_msg_ex(uint32_t id, uint32_t ide, uint32_t rtr, uint8_t *msg, uint8_t len)
{
    uint32_t tickstart;
    uint32_t TxMailbox = CAN_TX_MAILBOX0;
    uint32_t tx_status;
    uint32_t mailbox_shift;

    if (msg == NULL)
    {
        msg = g_can_dummy_data;
    }

    if (len > 8)
    {
        len = 8;
    }

    if (ide == CAN_ID_STD)
    {
        g_canx_txheader.StdId = id & 0x7FF;         /* 标准标识符 */
        g_canx_txheader.ExtId = 0;
    }
    else
    {
        g_canx_txheader.StdId = 0;
        g_canx_txheader.ExtId = id & 0x1FFFFFFF;    /* 扩展标识符(29位) */
    }

    g_canx_txheader.IDE = ide;          /* 标准帧/扩展帧 */
    g_canx_txheader.RTR = rtr;          /* 数据帧/远程帧 */
    g_canx_txheader.DLC = len;

    if (HAL_CAN_AddTxMessage(&g_canx_handler, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* 发送消息 */
    {
        return 1;                       /* 三个发送邮箱全满 */
    }

    /* 等待报文真正发出去(等所有邮箱为空)。总线上没有其他节点应答时会超时 */
    tickstart = HAL_GetTick();

    while (HAL_CAN_IsTxMessagePending(&g_canx_handler, TxMailbox))
    {
        if ((HAL_GetTick() - tickstart) > CAN_TX_TIMEOUT_MS)
        {
            HAL_CAN_AbortTxRequest(&g_canx_handler, TxMailbox);     /* 超时，直接中止邮箱的发送请求 */
            return 1;
        }
    }

    /* Each mailbox has its own completion and TXOK bits in TSR. */
    mailbox_shift = POSITION_VAL(TxMailbox) * 8U;
    tx_status = READ_REG(g_canx_handler.Instance->TSR);
    WRITE_REG(g_canx_handler.Instance->TSR, CAN_TSR_RQCP0 << mailbox_shift);
    return ((tx_status & (CAN_TSR_TXOK0 << mailbox_shift)) != 0U) ? 0 : 1;
}

/**
 * @brief       从软件接收队列取一帧报文
 * @param       msg     : 报文缓存
 * @retval      0, 队列为空; 1, 取到一帧
 *   @note      应用层推荐用这个, CANopen一个节点需要处理多个COB-ID
 */
uint8_t can_recv_msg(can_msg_t *msg)
{
#if !CAN_RX0_INT_ENABLE
    can_rx_fifo_to_queue();             /* 轮询模式: 主动把硬件FIFO搬进软件队列 */
#endif

    if (g_can_rx_tail == g_can_rx_head)
    {
        return 0;
    }

    *msg = g_can_rx_queue[g_can_rx_tail];
    g_can_rx_tail = (uint16_t)((g_can_rx_tail + 1) & (CAN_RX_QUEUE_DEPTH - 1));

    return 1;
}

/**
 * @brief       CAN 接收数据查询(兼容旧接口)
 *   @note      只找指定ID的标准数据帧, 队列中的其它报文会被丢弃
 * @param       id      : 要查询的 标准ID(11位)
 * @param       buf     : 数据缓存区
 * @retval      接收结果
 *   @arg       0   , 无数据被接收到;
 *   @arg       其他, 接收的数据长度
 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf)
{
    can_msg_t msg;
    uint8_t i;

    while (can_recv_msg(&msg))      /* 队列里有报文就一直找 */
    {
        if ((msg.id == id) && (msg.ide == CAN_ID_STD) && (msg.rtr == CAN_RTR_DATA))   /* 接收到的ID不对 / 不是标准帧 / 不是数据帧 */
        {
            for (i = 0; i < msg.dlc; i++)
            {
                buf[i] = msg.data[i];
            }

            return msg.dlc;
        }
    }

    return 0;
}

/**
 * @brief       读取CAN错误码
 * @param       无
 * @retval      HAL_CAN_GetError()的错误码, 0表示正常
 *   @note      典型值: CAN_ERROR_BIT(位错误)、CAN_ERROR_ACK(无节点应答)、
 *              CAN_ERROR_STUFF(位填充)、CAN_ERROR_CRC; 可用于定位接线/终端电阻问题
 */
uint32_t can_get_error(void)
{
    return HAL_CAN_GetError(&g_canx_handler);
}
