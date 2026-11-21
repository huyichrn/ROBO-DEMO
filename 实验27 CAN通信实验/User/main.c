/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V2.0
 * @date        2023-06-06
 * @brief       CAN 一主一从通信实验(CANopen 风格)
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
 * 实验说明:
 * 1, 两块板子, CAN_H接CAN_H, CAN_L接CAN_L, 并且必须共地; 总线上要有2个120欧终端电阻。
 * 2, 一块板子把 NODE_ROLE_MASTER 改成 1 编为主节点, 另一块保持 0 编为从节点(NODE_ID=1)。
 * 3, 通信内容完全按照 CANopen 的 COB-ID 规则和 SDO 帧格式组织:
 *      NMT      0x000        主->从   网络管理(启动/停止)
 *      SYNC     0x080        主->从   同步(广播)
 *      SDO请求  0x600+NodeId 主->从   读写对象字典
 *      SDO响应  0x580+NodeId 从->主   数据/写确认/中止
 *      PDO1     0x180+NodeId 从->主   8字节过程数据
 *      心跳     0x700+NodeId 从->主   节点状态
 *    下一阶段接真正的 CANopen 时, ID 和帧格式都不用改, 只把这里的简化解析换成完整协议。
 *
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"
#include "./BSP/CAN/can.h"
#include <string.h>

/* ================================================================================================
 * 一、本节点配置(烧录前修改这里)
 * ================================================================================================ */
#define NODE_ROLE_MASTER        0           /* 1: 主节点; 0: 从节点 */
#ifndef NODE_ID
#define NODE_ID                 1           /* 从节点ID, 范围 1~127, 主节点可忽略 */
#endif

#if NODE_ID < 1 || NODE_ID > 127
#error NODE_ID must be in the range 1 to 127
#endif

#define HEARTBEAT_MS            100         /* 心跳周期(ms) */
#define SYNC_PERIOD_MS          100         /* 主节点发SYNC的周期(ms) */
#define SDO_POLL_MS             250         /* 主节点轮询对象字典的周期(ms) */
#define APP_PRINT_EN            1           /* 1: 串口打印关键事件(不打印周期帧), 便于学习观察 */

/* ================================================================================================
 * 二、COB-ID 定义(与 CANopen 标准一致)
 * ================================================================================================ */
#define COB_NMT                 0x000U              /* 网络管理: 主 -> 从 */
#define COB_SYNC                0x080U              /* 同步对象: 主 -> 从(广播) */
#define COB_TIME                0x100U              /* 时间戳 */
#define COB_PDO1_TX(nid)        (0x180U + (nid))    /* 过程数据: 从 -> 主 */
#define COB_PDO1_RX(nid)        (0x200U + (nid))    /* 过程数据: 主 -> 从 */
#define COB_SDO_TX(nid)         (0x580U + (nid))    /* SDO响应: 从 -> 主 */
#define COB_SDO_RX(nid)         (0x600U + (nid))    /* SDO请求: 主 -> 从 */
#define COB_HEARTBEAT(nid)      (0x700U + (nid))    /* 心跳: 从 -> 主 */

/* NMT 命令 */
#define NMT_CMD_START           0x01                /* 进入运行态 */
#define NMT_CMD_STOP            0x02                /* 进入停止态 */
#define NMT_CMD_ENTER_PREOP     0x80                /* 进入预运行态 */

/* NMT 状态(心跳报文第一个字节) */
#define NMT_STATE_BOOTUP        0x00
#define NMT_STATE_STOPPED       0x04
#define NMT_STATE_OPERATIONAL   0x05
#define NMT_STATE_PREOP         0x7F

/* SDO 命令字: 请求(主->从) */
#define SDO_CCS_DOWNLOAD_4B     0x23                /* 写4字节 */
#define SDO_CCS_DOWNLOAD_2B     0x2B                /* 写2字节 */
#define SDO_CCS_DOWNLOAD_1B     0x2F                /* 写1字节 */
#define SDO_CCS_UPLOAD          0x40                /* 读请求 */

/* SDO 命令字: 响应(从->主) */
#define SDO_SCS_UPLOAD_4B       0x43                /* 读了4字节 */
#define SDO_SCS_UPLOAD_3B       0x47
#define SDO_SCS_UPLOAD_2B       0x4B                /* 读了2字节 */
#define SDO_SCS_UPLOAD_1B       0x4F                /* 读了1字节 */
#define SDO_SCS_DOWNLOAD_ACK    0x60                /* 写成功 */
#define SDO_ABORT               0x80                /* 中止 */

/* SDO 中止码 */
#define SDO_ABORT_NO_OBJECT     0x06020000U         /* 对象不存在 */
#define SDO_ABORT_READ_ONLY     0x06010002U         /* 试图写只读对象 */

/* ================================================================================================
 * 三、对象字典索引(先用手写变量模拟, 第2阶段换成真正的对象字典表)
 * ================================================================================================ */
#define OD_COUNTER              0x2000U             /* uint16, 只读: 运行计数 */
#define OD_KEYCOUNT             0x2001U             /* uint32, 可读可写: 按键计数 */

/* ================================================================================================
 * 四、全局变量
 * ================================================================================================ */
static uint32_t g_tx_cnt = 0;           /* 已发送帧数 */
static uint32_t g_rx_cnt = 0;           /* 已接收帧数 */
static uint32_t g_err_cnt = 0;          /* 发送失败次数(可用于判断总线是否正常) */

#if NODE_ROLE_MASTER

static uint32_t g_last_sync = 0;        /* 上次发SYNC的时刻 */
static uint32_t g_last_nmt = 0;         /* 上次发NMT的时刻 */
static uint32_t g_last_sdo = 0;         /* 上次发SDO请求的时刻 */
static uint32_t g_hb_tick = 0;          /* 最后一次收到心跳的时刻 */
static uint8_t  g_node_online = 0;      /* 从机在线标志 */
static uint8_t  g_sdo_turn = 0;         /* SDO轮询对象切换 */
static uint32_t g_counter_val = 0;      /* 从 0x2000 读回的值 */
static uint32_t g_keycnt_val = 0;       /* 从 0x2001 读回的值 */
static uint32_t g_write_val = 0;        /* KEY0按下后写入 0x2001 的值 */
static uint32_t g_sdo_ok = 0;           /* SDO成功次数 */
static uint32_t g_sdo_fail = 0;         /* SDO失败/中止次数 */

#else

static volatile uint8_t g_node_state = NMT_STATE_PREOP;   /* 本节点NMT状态, 上电后处于预运行态 */
static volatile uint16_t g_od_counter = 0;                   /* 对象 0x2000 */
static volatile uint32_t g_od_keycnt = 0;                    /* 对象 0x2001 */
static volatile uint32_t g_sync_cnt = 0;                     /* 收到的SYNC个数 */
#if !defined(CAN_TEST_HEADLESS) && !defined(CAN_TEST_NO_HEARTBEAT)
static uint32_t g_last_hb = 0;                               /* 上次发心跳的时刻 */
#endif

#endif

/**************************************************************************************************
 * 五、通用工具
 **************************************************************************************************/

/**
 * @brief       CAN发送封装, 统一统计收发帧数和错误数
 * @param       id      : 标准ID
 * @param       data    : 数据指针, 可为NULL(如SYNC)
 * @param       len     : 数据长度 0~8
 * @retval      0, 成功; 1, 失败
 */
#ifndef CAN_TEST_HEADLESS
static can_msg_t g_last_rx;
static can_msg_t g_last_tx;
static uint8_t g_lcd_rx_pending = 1;
static uint8_t g_lcd_tx_pending = 1;
static uint8_t g_last_tx_status;
#endif

static uint8_t app_can_send(uint32_t id, uint8_t *data, uint8_t len)
{
    uint8_t status = can_send_msg(id, data, len);

#ifndef CAN_TEST_HEADLESS
    g_last_tx.id = id & 0x7FF;
    g_last_tx.ide = CAN_ID_STD;
    g_last_tx.rtr = CAN_RTR_DATA;
    g_last_tx.dlc = (len > sizeof(g_last_tx.data)) ? sizeof(g_last_tx.data) : len;
    if (data != NULL && g_last_tx.dlc != 0)
    {
        memcpy(g_last_tx.data, data, g_last_tx.dlc);
    }
    else
    {
        memset(g_last_tx.data, 0, sizeof(g_last_tx.data));
    }
    g_last_tx_status = status;
    g_lcd_tx_pending = 1;
#endif

    if (status != 0)
    {
        g_err_cnt++;
        return 1;
    }

    g_tx_cnt++;
    return 0;
}

#if NODE_ROLE_MASTER
/**************************************************************************************************
 * 六、主节点: SDO 客户端(负责发请求、解析响应)
 **************************************************************************************************/

/**
 * @brief       发SDO读请求(上传对象字典)
 * @param       index   : 对象索引
 * @param       sub     : 子索引
 * @retval      无
 */
static void master_sdo_read(uint16_t index, uint8_t sub)
{
    uint8_t d[8] = {0};

    d[0] = SDO_CCS_UPLOAD;
    d[1] = (uint8_t)(index & 0xFF);
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;

    app_can_send(COB_SDO_RX(NODE_ID), d, 8);
}

/**
 * @brief       发SDO写请求(下载4字节到对象字典)
 * @param       index   : 对象索引
 * @param       sub     : 子索引
 * @param       value   : 要写入的值
 * @retval      无
 */
static void master_sdo_write_u32(uint16_t index, uint8_t sub, uint32_t value)
{
    uint8_t d[8];

    d[0] = SDO_CCS_DOWNLOAD_4B;
    d[1] = (uint8_t)(index & 0xFF);
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;
    d[4] = (uint8_t)(value & 0xFF);
    d[5] = (uint8_t)((value >> 8) & 0xFF);
    d[6] = (uint8_t)((value >> 16) & 0xFF);
    d[7] = (uint8_t)((value >> 24) & 0xFF);

    app_can_send(COB_SDO_RX(NODE_ID), d, 8);
}

/**
 * @brief       主节点收帧处理
 * @param       msg : 收到的报文
 * @retval      无
 */
static void master_rx_process(can_msg_t *msg)
{
    uint16_t index;
    uint8_t  sub;
    uint32_t value;

    if (msg->ide != CAN_ID_STD || msg->rtr != CAN_RTR_DATA)
    {
        return;
    }

    if (msg->id == COB_SDO_TX(NODE_ID))         /* SDO 响应 */
    {
        if (msg->dlc < 8)
        {
            return;
        }

        index = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
        sub = msg->data[3];

        value = (uint32_t)msg->data[4] | ((uint32_t)msg->data[5] << 8) |
                ((uint32_t)msg->data[6] << 16) | ((uint32_t)msg->data[7] << 24);

        if (msg->data[0] == SDO_ABORT)
        {
            g_sdo_fail++;

#if APP_PRINT_EN
            printf("SDO abort: idx=0x%04X sub=%d code=0x%08X\r\n", index, sub, value);
#endif
        }
        else if (msg->data[0] == SDO_SCS_DOWNLOAD_ACK)
        {
            g_sdo_ok++;

#if APP_PRINT_EN
            printf("SDO write OK: idx=0x%04X sub=%d\r\n", index, sub);
#endif
        }
        else if ((msg->data[0] == SDO_SCS_UPLOAD_2B) && (index == OD_COUNTER))
        {
            g_counter_val = value & 0xFFFF;
            g_sdo_ok++;
        }
        else if ((msg->data[0] == SDO_SCS_UPLOAD_4B) && (index == OD_KEYCOUNT))
        {
            g_keycnt_val = value;
            g_sdo_ok++;
        }
    }
    else if (msg->id == COB_HEARTBEAT(NODE_ID))   /* 心跳: 数据第1字节是NMT状态 */
    {
        g_hb_tick = HAL_GetTick();
        g_node_online = 1;

#if APP_PRINT_EN
        printf("Heartbeat: state=0x%02X\r\n", msg->data[0]);
#endif
    }
    else if (msg->id == COB_PDO1_TX(NODE_ID))     /* 过程数据 */
    {
        if (msg->dlc >= 6)
        {
            g_hb_tick = HAL_GetTick();
            g_node_online = 1;
            g_counter_val = (uint32_t)msg->data[0] | ((uint32_t)msg->data[1] << 8);
            g_keycnt_val = (uint32_t)msg->data[2] | ((uint32_t)msg->data[3] << 8) |
                           ((uint32_t)msg->data[4] << 16) | ((uint32_t)msg->data[5] << 24);
        }
    }
}

/**
 * @brief       主节点周期任务: NMT / SYNC / SDO轮询 / 心跳超时检测
 * @param       无
 * @retval      无
 */
static void master_task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  d[2];

    /* 1. 每2秒广播一次 NMT 启动, 让从机进入运行态(一主多从时把这帧发给所有节点) */
    if ((uint32_t)(now - g_last_nmt) >= 2000)
    {
        g_last_nmt = now;
        d[0] = NMT_CMD_START;
        d[1] = 0x00;                            /* 0 = 所有节点; 多从时可填具体 NODE_ID */
        app_can_send(COB_NMT, d, 2);
    }

    /* 2. 周期广播 SYNC, 从机收到后回一帧 PDO */
    if ((uint32_t)(now - g_last_sync) >= SYNC_PERIOD_MS)
    {
        g_last_sync = now;
        app_can_send(COB_SYNC, 0, 0);
    }

    /* 3. 轮流读取从机的两个对象, 验证 SDO 上传 */
    if ((uint32_t)(now - g_last_sdo) >= SDO_POLL_MS)
    {
        g_last_sdo = now;

        if (g_sdo_turn)
        {
            master_sdo_read(OD_KEYCOUNT, 0);
        }
        else
        {
            master_sdo_read(OD_COUNTER, 0);
        }

        g_sdo_turn = !g_sdo_turn;
    }

    /* 4. 心跳超时: 3个心跳周期收不到就判离线(一主多从的关键) */
    if (g_node_online && ((uint32_t)(now - g_hb_tick) > (3 * HEARTBEAT_MS)))
    {
        g_node_online = 0;
    }
}

#else
/**************************************************************************************************
 * 七、从节点: SDO 服务器 + NMT 状态机 + 心跳 + PDO
 **************************************************************************************************/

/**
 * @brief       从节点回SDO中止响应
 * @param       index   : 对象索引
 * @param       sub     : 子索引
 * @param       code    : 中止码
 * @retval      无
 */
static void sdo_send_abort(uint16_t index, uint8_t sub, uint32_t code)
{
    uint8_t d[8];

    d[0] = SDO_ABORT;
    d[1] = (uint8_t)(index & 0xFF);
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;
    d[4] = (uint8_t)(code & 0xFF);
    d[5] = (uint8_t)((code >> 8) & 0xFF);
    d[6] = (uint8_t)((code >> 16) & 0xFF);
    d[7] = (uint8_t)((code >> 24) & 0xFF);

    app_can_send(COB_SDO_TX(NODE_ID), d, 8);
}

/**
 * @brief       从节点回SDO上传响应(读对象字典的结果)
 * @param       index   : 对象索引
 * @param       sub     : 子索引
 * @param       value   : 数据
 * @param       size    : 字节数 1/2/4
 * @retval      无
 */
static void sdo_send_upload(uint16_t index, uint8_t sub, uint32_t value, uint8_t size)
{
    uint8_t d[8];

    d[0] = (size == 1) ? SDO_SCS_UPLOAD_1B : ((size == 2) ? SDO_SCS_UPLOAD_2B : SDO_SCS_UPLOAD_4B);
    d[1] = (uint8_t)(index & 0xFF);
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;
    d[4] = (uint8_t)(value & 0xFF);
    d[5] = (uint8_t)((value >> 8) & 0xFF);
    d[6] = (uint8_t)((value >> 16) & 0xFF);
    d[7] = (uint8_t)((value >> 24) & 0xFF);

    app_can_send(COB_SDO_TX(NODE_ID), d, 8);
}

/**
 * @brief       从节点处理SDO请求
 * @param       msg : 收到的SDO请求报文
 * @retval      无
 */
static void sdo_server_process(can_msg_t *msg)
{
    uint8_t  cmd;
    uint16_t index;
    uint8_t  sub;
    uint32_t value;
    uint8_t  d[8];

    if (msg->dlc < 8)
    {
        return;
    }

    cmd   = msg->data[0];
    index = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
    sub   = msg->data[3];
    value = (uint32_t)msg->data[4] | ((uint32_t)msg->data[5] << 8) |
            ((uint32_t)msg->data[6] << 16) | ((uint32_t)msg->data[7] << 24);

    if (cmd == SDO_CCS_UPLOAD)                      /* 读对象 */
    {
        if ((index == OD_COUNTER) && (sub == 0))
        {
            sdo_send_upload(index, sub, g_od_counter, 2);
        }
        else if ((index == OD_KEYCOUNT) && (sub == 0))
        {
            sdo_send_upload(index, sub, g_od_keycnt, 4);
        }
        else
        {
            sdo_send_abort(index, sub, SDO_ABORT_NO_OBJECT);
        }
    }
    else if ((cmd == SDO_CCS_DOWNLOAD_4B) || (cmd == SDO_CCS_DOWNLOAD_2B) ||
             (cmd == SDO_CCS_DOWNLOAD_1B))          /* 写对象 */
    {
        if ((index == OD_KEYCOUNT) && (sub == 0))
        {
            g_od_keycnt = value;                    /* 更新对象字典 */
            d[0] = SDO_SCS_DOWNLOAD_ACK;            /* 写成功: 回 0x60 */
            d[1] = (uint8_t)(index & 0xFF);
            d[2] = (uint8_t)(index >> 8);
            d[3] = sub;
            d[4] = 0;
            d[5] = 0;
            d[6] = 0;
            d[7] = 0;
            app_can_send(COB_SDO_TX(NODE_ID), d, 8);

#if APP_PRINT_EN
            printf("SDO write: 0x%04X = %lu\r\n", index, (unsigned long)value);
#endif
        }
        else if ((index == OD_COUNTER) && (sub == 0))
        {
            sdo_send_abort(index, sub, SDO_ABORT_READ_ONLY);    /* 只读对象 */
        }
        else
        {
            sdo_send_abort(index, sub, SDO_ABORT_NO_OBJECT);
        }
    }
    else
    {
        sdo_send_abort(index, sub, SDO_ABORT_NO_OBJECT);
    }
}

/**
 * @brief       从节点收帧处理: NMT / SYNC / SDO
 * @param       msg : 收到的报文
 * @retval      无
 */
static void slave_rx_process(can_msg_t *msg)
{
    uint8_t d[8];

    if (msg->ide != CAN_ID_STD || msg->rtr != CAN_RTR_DATA)
    {
        return;
    }

    if (msg->id == COB_NMT)                         /* 网络管理: 切换NMT状态 */
    {
        if ((msg->dlc >= 2) && ((msg->data[1] == 0x00) || (msg->data[1] == NODE_ID)))
        {
            if (msg->data[0] == NMT_CMD_START)
            {
                g_node_state = NMT_STATE_OPERATIONAL;
            }
            else if (msg->data[0] == NMT_CMD_STOP)
            {
                g_node_state = NMT_STATE_STOPPED;
            }
            else if (msg->data[0] == NMT_CMD_ENTER_PREOP)
            {
                g_node_state = NMT_STATE_PREOP;
            }

#if APP_PRINT_EN
            printf("NMT: cmd=0x%02X state=0x%02X\r\n", msg->data[0], g_node_state);
#endif
        }
    }
    else if (msg->id == COB_SYNC)                   /* 同步: 计数 + 运行态回PDO */
    {
        g_sync_cnt++;

        if (g_node_state == NMT_STATE_OPERATIONAL)
        {
            d[0] = (uint8_t)(g_od_counter & 0xFF);
            d[1] = (uint8_t)(g_od_counter >> 8);
            d[2] = (uint8_t)(g_od_keycnt & 0xFF);
            d[3] = (uint8_t)((g_od_keycnt >> 8) & 0xFF);
            d[4] = (uint8_t)((g_od_keycnt >> 16) & 0xFF);
            d[5] = (uint8_t)((g_od_keycnt >> 24) & 0xFF);
            d[6] = 0;
            d[7] = 0;
            app_can_send(COB_PDO1_TX(NODE_ID), d, 8);   /* PDO1: 从 -> 主 */
        }
    }
    else if (msg->id == COB_SDO_RX(NODE_ID))        /* 本节点的SDO请求 */
    {
        sdo_server_process(msg);
    }
}

/**
 * @brief       从节点周期任务: 心跳
 * @param       无
 * @retval      无
 */
#if !defined(CAN_TEST_HEADLESS) && !defined(CAN_TEST_NO_HEARTBEAT)
static void slave_task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  d[1];

    if ((uint32_t)(now - g_last_hb) >= HEARTBEAT_MS)
    {
        g_last_hb = now;
        d[0] = g_node_state;
        app_can_send(COB_HEARTBEAT(NODE_ID), d, 1);
    }
}
#endif

#endif

/**************************************************************************************************
 * 八、LCD 显示
 **************************************************************************************************/

/**
 * @brief       画固定不变的提示文字(只在上电时画一次)
 * @param       无
 * @retval      无
 */
#ifndef CAN_TEST_HEADLESS
static void lcd_show_frame(uint16_t y, const can_msg_t *frame, uint32_t count,
                           uint16_t color)
{
    static const char hex[] = "0123456789ABCDEF";
    char header[28];
    char bytes[24];
    uint8_t i;

    if (count == 0)
    {
        lcd_show_string(10, y, 220, 16, 16, "Waiting for frame", color);
        return;
    }

    if (frame->ide == CAN_ID_EXT)
    {
        snprintf(header, sizeof(header), "EXT %08lX DLC %u",
                 (unsigned long)frame->id, (unsigned int)frame->dlc);
    }
    else
    {
        snprintf(header, sizeof(header), "STD %03lX DLC %u",
                 (unsigned long)frame->id, (unsigned int)frame->dlc);
    }
    lcd_fill(10, y, 201, y + 35, WHITE);
    lcd_show_string(10, y, 220, 16, 16, header, color);

    if (frame->rtr == CAN_RTR_REMOTE || frame->dlc == 0)
    {
        lcd_show_string(10, y + 20, 220, 16, 16,
                        (frame->rtr == CAN_RTR_REMOTE) ? "Remote frame" : "No data", color);
        return;
    }

    for (i = 0; i < frame->dlc; i++)
    {
        bytes[3 * i] = hex[frame->data[i] >> 4];
        bytes[3 * i + 1] = hex[frame->data[i] & 0x0F];
        bytes[3 * i + 2] = ' ';
    }
    bytes[3 * i - 1] = '\0';
    lcd_show_string(10, y + 20, 220, 16, 16, bytes, color);
}

static void lcd_show_static(void)
{
    lcd_show_string(10, 16, 220, 16, 16, "CAN1 500 kbit/s", BLACK);
}

static void lcd_update(void)
{
    char label[27];

    if (g_lcd_rx_pending)
    {
        snprintf(label, sizeof(label), "RX  %10lu", (unsigned long)g_rx_cnt);
        lcd_show_string(10, 52, 220, 16, 16, label, BLUE);
        lcd_show_frame(72, &g_last_rx, g_rx_cnt, BLUE);
        g_lcd_rx_pending = 0;
    }
    if (g_lcd_tx_pending)
    {
        snprintf(label, sizeof(label), "TX  %10lu  %-5s",
                 (unsigned long)g_tx_cnt,
                 ((g_tx_cnt | g_err_cnt) == 0) ? "" : ((g_last_tx_status == 0) ? "OK" : "ERROR"));
        lcd_show_string(10, 126, 220, 16, 16, label, BLACK);
        lcd_show_frame(146, &g_last_tx, g_tx_cnt | g_err_cnt,
                       (g_last_tx_status == 0) ? BLACK : RED);
        g_lcd_tx_pending = 0;
    }
}
#endif

/**************************************************************************************************
 * 九、主函数
 **************************************************************************************************/
int main(void)
{
    uint8_t  key;
    uint32_t last_lcd = 0;
    can_msg_t rxmsg;
    uint8_t  res;

    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);     /* 设置时钟,168Mhz */
    delay_init(168);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口初始化为115200 */
    usmart_dev.init(84);                    /* 初始化USMART */
    led_init();                             /* 初始化LED */
#ifndef CAN_TEST_HEADLESS
    lcd_init();                             /* 初始化LCD */
#endif
    key_init();                             /* 初始化按键 */

    /*
     * CAN初始化, 波特率500Kbps, 采样点78.6%(42M / ((10 + 3 + 1) * 6) = 500K)
     * 注意: 普通模式必须有两个及以上节点才通信成功; 单板自测请把 CAN_MODE_NORMAL 改成 CAN_MODE_LOOPBACK
     */
    res = can_init(CAN_SJW_1TQ, CAN_BS2_3TQ, CAN_BS1_10TQ, 6, CAN_MODE_NORMAL);

    if (res != 0)
    {
        printf("CAN init failed: %u\r\n", (unsigned int)res);
#ifndef CAN_TEST_HEADLESS
        lcd_show_string(10, 20, 210, 16, 16, "CAN Init Failed!", RED);
        lcd_show_xnum(10, 40, res, 2, 16, 0x80, RED);
#endif

        while (1);
    }

#ifndef CAN_TEST_HEADLESS
    lcd_show_static();
#endif

#if APP_PRINT_EN
#if NODE_ROLE_MASTER
    printf("CAN node start: MASTER, target NodeId=%d, 500Kbps\r\n", NODE_ID);
#else
    printf("CAN node start: SLAVE, NodeId=%d, 500Kbps\r\n", NODE_ID);
#endif
#endif

    while (1)
    {
        key = key_scan(0);

        /* ------------------------------ 按键与周期任务 ------------------------------ */
#if NODE_ROLE_MASTER
        if (key == KEY0_PRES)                   /* 按下KEY0: 通过SDO写从机的 0x2001 对象 */
        {
            g_write_val++;
            master_sdo_write_u32(OD_KEYCOUNT, 0, g_write_val);

#if APP_PRINT_EN
            printf("SDO write req: 0x2001 = %lu\r\n", (unsigned long)g_write_val);
#endif
        }

        master_task();
#else
        if (key == KEY0_PRES)                   /* 按下KEY0: 本地修改 0x2001 对象, 等主节点读回 */
        {
            g_od_keycnt++;
        }

#if !defined(CAN_TEST_HEADLESS) && !defined(CAN_TEST_NO_HEARTBEAT)
        slave_task();
#endif
#endif

        /* ------------------------------ 收取CAN报文 ------------------------------ */
        while (can_recv_msg(&rxmsg))
        {
            g_rx_cnt++;
#ifndef CAN_TEST_HEADLESS
            g_last_rx = rxmsg;
            g_lcd_rx_pending = 1;
#endif
#if APP_PRINT_EN
            printf("CAN RX #%lu ID=0x%lX DLC=%u\r\n",
                   (unsigned long)g_rx_cnt, (unsigned long)rxmsg.id, (unsigned int)rxmsg.dlc);
#endif

#if NODE_ROLE_MASTER
            master_rx_process(&rxmsg);
#else
            slave_rx_process(&rxmsg);
#endif
        }

        /* ------------------------------ 定时刷新显示 ------------------------------ */
        if ((uint32_t)(HAL_GetTick() - last_lcd) >= 200)
        {
            last_lcd = HAL_GetTick();
#ifndef CAN_TEST_HEADLESS
            lcd_update();
#endif
            LED0_TOGGLE();                      /* 提示程序正在运行 */
        }

        delay_ms(2);
    }
}
