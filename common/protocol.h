/*
 * 2.4G Mouse-Dongle 共享协议定义
 *
 *  空中帧: [S0(1B)] [LEN(1B)] [S1(1B)] [PAYLOAD] [CRC(2B)]
 *  LFLEN=1, Radio 硬件自动处理长度字节
 *
 *  PAYLOAD[0] = TYPE (bit7: 0=noACK 1=needACK)
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ===== 状态机 ===== */
typedef enum {
	ST_IDLE,           /* 初始（TX专用）*/
	ST_PAIRING,        /* 配对中 */
	ST_RECONNECTING,   /* 回连中 */
	ST_CONNECTED,      /* 已连接 */
} link_state_t;

/* ===== TYPE 定义 ===== */
#define TYPE_MASK_ACK  0x80   /* bit7=1: 需要 ACK */

/* -- 数据类 (bit7=0, no ACK, 8K高速) -- */
#define TYPE_MOUSE      0x00
#define TYPE_KEYBOARD   0x01
#define TYPE_CONSUMER   0x02
#define TYPE_SYSTEM     0x03
/* 0x04~0x0F 预留 */

/* -- 控制类 (bit7=1, 需要 ACK) -- */
#define TYPE_WINDOW_OPEN    0x80
#define TYPE_CMD_DONGLE     0x81
#define TYPE_ACK            0x82
#define TYPE_PAIR_REQ       0x83
#define TYPE_PAIR_RESP      0x84
#define TYPE_PAIR_CONFIRM    0x85
#define TYPE_RECONN_REQ     0x86
#define TYPE_RECONN_RESP    0x87
#define TYPE_RSSI_REPORT    0x88
#define TYPE_BATTERY_REQ    0x89
#define TYPE_BATTERY_RESP   0x8A
#define TYPE_FW_VERSION_REQ 0x8B
#define TYPE_FW_VERSION_RESP 0x8C
#define TYPE_DPI_CHANGE     0x8D
#define TYPE_LED_CONFIG     0x8E
#define TYPE_SLEEP_CMD      0x8F
/* 0x90~0xBF 预留 */

/* -- CMD_DONGLE 的 cmd_id (payload[1]) -- */
#define CMD_SET_DPI          0x01
#define CMD_SET_LED          0x02
#define CMD_SET_POLLING      0x03
#define CMD_GET_BATTERY      0x04
#define CMD_GET_FW_VERSION   0x05
#define CMD_SLEEP            0x06
#define CMD_FW_UPGRADE_START 0x07
#define CMD_FW_DATA          0x08
#define CMD_RESET            0x09

/* ===== 超时参数 ===== */
#define PAIR_TIMEOUT_S       30
#define RECONN_TIMEOUT_S     30
#define LONG_PRESS_MS        3000
#define ACK_TIMEOUT_US       500
#define ACK_MAX_RETRIES      10
#define RECONN_MAX_RETRIES   3

/* ===== WINDOW 参数 ===== */
#define WINDOW_INTERVAL_US   10000   /* 每 10ms 开一次窗口 */
#define WINDOW_DURATION_US   500     /* 开窗时长 */

/* ===== 配对地址(公共) ===== */
/* Radio BASE0 = 0x01234567, 前缀见 radio_config */

/* ===== Flash 存储 Key ===== */
#define NVS_PAIRED_FLAG  0x01
#define NVS_PEER_ADDR    0x02
#define NVS_MY_ADDR      0x03

/* ===== 工具宏 ===== */
static inline bool type_needs_ack(uint8_t t) { return (t & TYPE_MASK_ACK) != 0; }
static inline bool type_is_data(uint8_t t)  { return (t & TYPE_MASK_ACK) == 0; }

#endif /* PROTOCOL_H */
