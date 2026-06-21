# nRF54L20 2.4G ESB 工程详解

> 基于 Nordic nRF Connect SDK v3.3.0 官方 Enhanced ShockBurst (ESB) 例程。
> 从键鼠行业嵌入式开发视角，逐行拆解 2.4G 私有协议通讯逻辑。

---

## 目录

1. [工程概览](#1-工程概览)
2. [ESB 协议核心概念](#2-esb-协议核心概念)
3. [TX (PTX) 逐行解析](#3-tx-ptx-逐行解析)
4. [RX (PRX) 逐行解析](#4-rx-prx-逐行解析)
5. [TX 与 RX 的关键差异](#5-tx-与-rx-的关键差异)
6. [从例程到产品：键鼠应用改造思路](#6-从例程到产品键鼠应用改造思路)
7. [编译与烧录](#7-编译与烧录)

---

## 1. 工程概览

```
nRF54lm20_24G/
├── tx/                          # PTX (Primary Transmitter) —— 发送端
│   ├── CMakeLists.txt           # Zephyr CMake 构建脚本
│   ├── Kconfig                  # 应用级 Kconfig（日志等级）
│   ├── Kconfig.sysbuild         # sysbuild 分区管理
│   ├── prj.conf                 # 运行时配置（协议栈开关）
│   ├── sysbuild.cmake           # 多镜像构建（nRF54L20 单核，已简化）
│   └── src/
│       └── main.c               # ★ 全部业务逻辑（~160 行）
└── rx/                          # PRX (Primary Receiver) —— 接收端
    ├── ... (同上结构)
    └── src/
        └── main.c               # ★ 全部业务逻辑（~160 行）
```

**两个工程共用同一套 ESB 地址配置**，互相对发。TX 发数据包 → RX 回 ACK 包，构成双向链路。

---

## 2. ESB 协议核心概念

### 2.1 什么是 Enhanced ShockBurst？

ESB 是 Nordic 的**私有 2.4G 协议**，构建在 nRF Radio 硬件之上。相比于 BLE，它：

| 特性 | ESB | BLE |
|------|-----|-----|
| 空中速率 | 250kbps / 1Mbps / **2Mbps** | 1Mbps / 2Mbps |
| 双向通讯 | ACK payload（自带） | GATT Write/Notify |
| 协议栈大小 | ~5KB Flash | ~80KB+ Flash (SoftDevice) |
| 延迟 | **极低**（~100μs 级） | 连接间隔 7.5ms~4s |
| 配对 | 无标准流程（需自定义） | SMP 标准配对 |
| 功耗 | 较低（无持续广播） | 广播/连接间隔可调 |

**键鼠行业的启示**：ESB 是罗技 Lightspeed / G Series 等私有 2.4G 协议的同类型技术——简单、低延迟、私有地址、无蓝牙协议栈开销。官方例程只演示基础收发，实际上你可以用完全相同的内核做出 8K 回报率的游戏鼠标。

### 2.2 核心概念

#### 地址体系（Addressing）

ESB 的空中地址由三部分组成：

```
  ┌────────────────────────────────────────────┐
  │  Base Address (4 bytes)                    │  ← 公共部分，所有 pipe 共用
  │  + Prefix (1 byte)                        │  ← 区分不同 pipe/设备
  │  = On-air Address (5 bytes total)         │
  └────────────────────────────────────────────┘
```

- **Base Address 0**：pipe 0~7 共用（例程：`0xE7E7E7E7`）
- **Base Address 1**：pipe 1~7 可选另一组基地址（例程：`0xC2C2C2C2`）
- **8 个 Prefix**：每个 pipe 一个字节，组成完整 5 字节地址
- **例程中的完整地址表**：

| Pipe | Base | Prefix | 空中地址 |
|------|------|--------|----------|
| 0 | 0xE7E7E7E7 | 0xE7 | 0xE7E7E7E7E7 |
| 1 | 0xC2C2C2C2 | 0xC2 | 0xC2C2C2C2C2 |
| 2 | 0xC2C2C2C2 | 0xC3 | 0xC3C2C2C2C2 |
| ... | ... | ... | ... |

> **量产注意**：例程用的是写死的默认地址。产品中每套键鼠应该协商出独立地址，避免多套设备互相干扰。

#### Pipe（逻辑通道）

- PRX 端最多监听 **8 个 pipe**（每个 pipe 有独立地址）
- PTX 端用指定 pipe 发送
- 可以理解为"子地址"：一个 dongle 同时和多个设备通讯（如键盘 + 鼠标用不同 pipe）

#### ACK Payload

ESB 的 ACK 不是空帧——它可以**携带数据**：

```
PTX 发送 data ───────────────────────────────→ PRX 收到 data
PTX 收到 ack_data ←─────────────────────────── PRX 附带回数据
```

这就是 ESB 实现双向通讯的基础，极低开销。

#### DPL (Dynamic Payload Length)

- 传统 ESB：所有包固定长度
- **ESB DPL**：每包可变长度（1~252 字节），更灵活
- 例程中 `config.protocol = ESB_PROTOCOL_ESB_DPL` 即启用此模式

#### Selective Auto-ACK

允许发送方**按包决定**是否需要 ACK：
- `tx_payload.noack = false` → 要 ACK（可靠传输）
- `tx_payload.noack = true` → 不要 ACK（低延迟，如鼠标位移丢一帧无所谓）

### 2.3 数据流时序

```
时间轴 →

TX 端 (PTX):
  写 TX FIFO ──→ Radio 自动发送 ──→ 等 ACK
                                      ├── 收到 ACK → ESB_EVENT_TX_SUCCESS → ready=true
                                      └── 超时     → 自动重传（最多 N 次）
                                                   └── 耗尽 → ESB_EVENT_TX_FAILED → ready=true

RX 端 (PRX):
  持续监听 ──→ 收到包 → ESB_EVENT_RX_RECEIVED
                       └── 自动回 ACK（如果 noack=false）
                       └── 如果 TX FIFO 有数据 → 附在 ACK 里发出（ACK payload）
```

**关键**：TX 端写一包后必须等 `ready` 才能写下一包——这是 ESB 的流控机制。

---

## 3. TX (PTX) 逐行解析

### 3.1 Include 分析

```c
#include <zephyr/devicetree.h>    // 设备树
#include <zephyr/logging/log.h>   // Zephyr 日志系统
#include <esb.h>                  // ★ ESB 协议栈头文件（核心）
#include <zephyr/kernel.h>        // k_sleep()
#include <zephyr/types.h>         // 类型定义
#include <zephyr/pm/device_runtime.h>  // 电源管理（nRF54H 系列需要）
#include <dk_buttons_and_leds.h>  // DK 板载 LED
```

### 3.2 模块注册 + 状态变量

```c
LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);
//                   ↑ 模块名  ↑ 日志等级（Kconfig 中 default=4 → DEBUG）

static bool ready = true;   // ★ TX 流控标志
                            // true  = 可以写下一包
                            // false = 等待上一包 ACK/超时
```

`ready` 变量是整个发送循环的核心——**它是手动的、单包流控**。

### 3.3 Payload 初始化

```c
static struct esb_payload rx_payload;  // 接收 ACK 回包用的缓冲区

static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
    0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);
//  ↑ pipe  ↑────── 8 字节 payload 数据 ──────────↑
```

`ESB_CREATE_PAYLOAD(pipe, byte0, byte1, ...)` 是一个宏，生成 `esb_payload` 结构体：

```c
// esb_payload 结构体（简化）
struct esb_payload {
    uint8_t pipe;       // 目标 pipe（0~7）
    uint8_t length;     // payload 长度（DPL 模式可变）
    uint8_t data[252];  // 数据缓冲区
    bool    noack;      // 是否跳过 ACK
};
```

### 3.4 事件回调 —— 通讯状态机的核心

```c
void event_handler(struct esb_evt const *event)
{
    ready = true;  // ★ 无论什么事件，都释放流控锁

    switch (event->evt_id) {
```

**ESB 所有事件都在 Radio 中断上下文中回调**（不是线程上下文！），所以：
- 不能做耗时操作（不要在里面 `k_sleep`）
- 不能调用需要线程上下文的 API
- 应该极快返回，只记状态/投事件

#### TX_SUCCESS 事件

```c
case ESB_EVENT_TX_SUCCESS:
    LOG_DBG("TX SUCCESS EVENT %u attempts", event->tx_attempts);
    break;
```

- 收到 ACK → `ready=true`（上面已设置）→ 主循环可以发下一包
- `event->tx_attempts` 告诉你重传了几次（1 = 一次成功，>1 = 有干扰/距离远）

#### TX_FAILED 事件

```c
case ESB_EVENT_TX_FAILED:
    LOG_DBG("TX FAILED EVENT");
    break;
```

- 所有重传都失败了（默认最多 3 次，可配 `config.retransmit_count`）
- 包丢了，但是 `ready=true`，可以继续发下一包
- **键鼠建议**：对于重要数据（按键事件、DPI 切换），失败后应重发；对于位移数据，丢就丢了，下一包会覆盖

#### RX_RECEIVED 事件（PTX 端收到 ACK payload）

```c
case ESB_EVENT_RX_RECEIVED:
    while (esb_read_rx_payload(&rx_payload) == 0) {
        LOG_DBG("Packet received, len %d : 0x%02x, ...");
    }
    break;
```

- PTX 端也会收到数据——**ACK 里带的 payload**
- `while` 循环读空 FIFO（可能积了多包）
- 这个数据是 PRX 通过 `esb_write_payload` 写到 TX FIFO 里的（见 RX 端代码）
- **键鼠应用**：dongle 可以通过 ACK payload 向鼠标发送配置命令（改 DPI、改灯效等）

#### TIMESLOT_FAILED 事件

```c
#if IS_ENABLED(CONFIG_ESB_MPSL_TIMESLOT)
    case ESB_EVENT_TIMESLOT_FAILED:
        LOG_ERR("Error in Timeslot handling");
        break;
#endif
```

- 仅当 ESB 与 BLE 共用射频（MPSL timeslot）时有效
- 本例程未开此功能，不会触发

### 3.5 ESB 初始化 —— 最关键的配置段

```c
int esb_initialize(void)
{
    int err;

    // ===== 地址配置 =====
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};
```

**地址生成规则**：
- Pipe 0 地址 = base_addr_0 + prefix[0] = `0xE7E7E7E7` + `0xE7` = `0xE7E7E7E7E7` （5 字节）
- Pipe 1~7 地址 = base_addr_1 + prefix[1~7]
- TX 和 RX 的地址**必须完全一致**才能通讯

```c
    // ===== 协议配置 =====
    struct esb_config config = ESB_DEFAULT_CONFIG;  // 先用默认值

    config.protocol    = ESB_PROTOCOL_ESB_DPL;      // 动态 payload 长度
    config.retransmit_delay = 600;                  // 重传间隔 600μs
    config.bitrate     = ESB_BITRATE_2MBPS;         // 2Mbps 空速
    config.mode        = ESB_MODE_PTX;              // ★ 主发送模式
    config.event_handler = event_handler;           // 事件回调
    config.selective_auto_ack = true;               // 按 noack 标志决定要不要 ACK
```

`ESB_DEFAULT_CONFIG` 预填充了哪些值？

```
config.retransmit_count       = 3       // 最多重传 3 次
config.retransmit_delay       = 600     // 每次重传间隔 600μs
config.tx_mode                = AUTO    // 发送完自动切回 RX 等 ACK
config.payload_length         = 32      // 静态模式下的固定长度（DPL 模式忽略）
config.protocol               = DPL     // (宏默认)
config.bitrate                = 2MBPS   // (宏默认)
config.mode                   = PTX/PRX // 取决于编译哪个
config.selective_auto_ack     = false   // (宏默认)
config.interrupt_priority     = 1       // Radio 中断优先级
config.event_handler          = NULL
config.rf_channel             = 0       // 逻辑信道（0~100），映射到 2400~2500MHz
                                        //    0=2402MHz, 1=2403MHz, ... 100=2502MHz
```

> **重要**：`rf_channel` 默认 0（2402MHz）。产品中应做**跳频**（和 BLE 一样），在 0~100 之间按跳频表切换，躲避 WiFi 干扰。

```c
    // ===== 初始化 + 设地址 =====
    err = esb_init(&config);
    err = esb_set_base_address_0(base_addr_0);
    err = esb_set_base_address_1(base_addr_1);
    err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
```

注意：`esb_init` **必须在设地址之前调用**，因为 init 会复位 ESB 内部状态。

### 3.6 LED 更新逻辑

```c
static void leds_update(uint8_t value)
{
    uint32_t leds_mask =
        (!(value % 8 > 0 && value % 8 <= 4) ? DK_LED1_MSK : 0) |
        (!(value % 8 > 1 && value % 8 <= 5) ? DK_LED2_MSK : 0) |
        (!(value % 8 > 2 && value % 8 <= 6) ? DK_LED3_MSK : 0) |
        (!(value % 8 > 3) ? DK_LED4_MSK : 0);
    dk_set_leds(leds_mask);
}
```

这段用 `value` 的低 3 位控制 4 个 LED，做跑马灯效果：

| value % 8 | LED1 | LED2 | LED3 | LED4 | 视觉效果 |
|-----------|------|------|------|------|---------|
| 0 | ● | ○ | ○ | ○ | 单灯右移 |
| 1 | ● | ● | ○ | ○ | |
| 2 | ○ | ● | ● | ○ | |
| 3 | ○ | ○ | ● | ● | |
| 4 | ○ | ○ | ○ | ● | |
| 5 | ○ | ○ | ● | ● | |
| 6 | ○ | ● | ● | ○ | |
| 7 | ● | ● | ○ | ○ | |

### 3.7 main() —— 主循环

```c
int main(void)
{
    // 1. LED 初始化
    dk_leds_init();

    // 2. ESB 初始化
    esb_initialize();

    // 3. 主循环
    tx_payload.noack = false;    // 要 ACK（可靠传输）
    while (1) {
        if (ready) {             // ★ 等上一包 ACK/超时
            ready = false;
            esb_flush_tx();      // 清 TX FIFO（丢弃旧数据，保证发最新）
            leds_update(tx_payload.data[1]);  // LED 跑马灯
            esb_write_payload(&tx_payload);   // 写 FIFO → Radio 自动发送
            tx_payload.data[1]++;             // 计数器递增
        }
        k_sleep(K_MSEC(100));    // 100ms 间隔 → 10 包/秒
    }
}
```

**主循环流程图**：

```
         ┌──────────────────┐
         │   sleep 100ms    │←────────────┐
         └────────┬─────────┘             │
                  │                        │
             ┌────▼────┐                   │
             │ ready?  │── No ─────────────┘
             └────┬────┘
                  │ Yes
             ┌────▼────────────┐
             │  ready = false   │  ← 锁住，等 ACK
             │  flush TX FIFO   │  ← 弃旧，保证实时性
             │  LED 跑马灯      │
             │  write payload   │  ← 写入 TX FIFO
             │  counter++       │
             └────┬────────────┘
                  │
             ┌────▼────────────┐
             │  Radio 自动：    │
             │  发→等ACK→重传  │
             │  (后台硬件执行)  │
             └────┬────────────┘
                  │
      ┌───────────┴───────────┐
      │                       │
  ACK 收到              超时/失败
      │                       │
  ESB_EVENT_             ESB_EVENT_
  TX_SUCCESS             TX_FAILED
      │                       │
      └───────────┬───────────┘
                  │
          ready = true  ← 释放锁
```

### 3.8 关键设计点：esb_flush_tx()

```c
ready = false;
esb_flush_tx();          // ★ 清空 FIFO
esb_write_payload(&tx);  // 写新数据
```

为什么要在发之前 flush？因为：
- 如果上一包发了很久才等到 ACK（重传多次），期间调用 `esb_write_payload` 不会失败
- 但如果 ACK 后 `ready=true`，FIFO 里可能还有旧数据没发
- **键鼠场景**：位移数据有时效性，旧数据应该丢弃，只发最新的
- 这就是"coalescing"——合并/丢弃旧帧，保证实时性

---

## 4. RX (PRX) 逐行解析

RX 端大部分与 TX 端相同，下面只分析差异部分。

### 4.1 PRX 独有：esb_start_rx()

```c
err = esb_start_rx();
if (err) {
    LOG_ERR("RX setup failed, err %d", err);
    return 0;
}
```

**TX 端没有这个调用！** 这是 PRX 和 PTX 的根本差异：
- PTX：写 `esb_write_payload` → Radio 自动发送（RX 只在等 ACK 时短暂开启）
- **PRX**：必须调用 `esb_start_rx()` → Radio 持续处于 RX 模式 → 等待空中数据

### 4.2 PRX 的 TX FIFO = ACK Payload

```c
while (true) {
    err = esb_write_payload(&tx_payload);   // 不是"发送"！是"准备 ACK 数据"
    tx_payload.data[0]++;
    k_msleep(550);
}
```

**理解这个循环的关键**：PRX 端的 `esb_write_payload` **不是主动发送**！它是把数据写到 TX FIFO，当下次收到 PTX 的包并回 ACK 时，ESB 硬件会**自动把 TX FIFO 里的数据附在 ACK 帧后面**发出去。

```
PRX 端:
  写 TX FIFO ──→ 等待 PTX 发包 ──→ 收到 PTX 包 ──→ 回 ACK（附带 TX FIFO 数据）
                                                      └── ESB_EVENT_TX_SUCCESS
```

这就是 ESB 实现**双向通讯**的核心机制。PTX 发包 → PRX 收包 + 回数据；PTX 的 `ESB_EVENT_RX_RECEIVED` 拿到的就是 PRX 写到 TX FIFO 的数据。

### 4.3 PRX 的 RX_RECEIVED 处理

```c
case ESB_EVENT_RX_RECEIVED:
    while ((err = esb_read_rx_payload(&rx_payload)) == 0) {
        LOG_DBG("Packet received, len %d : 0x%02x, ...");
        leds_update(rx_payload.data[1]);  // 用收到的数据更新 LED
    }
    if (err && err != -ENODATA) {
        LOG_ERR("Error while reading rx packet");
    }
    break;
```

- `while` 循环读空 RX FIFO——如果中断处理来不及，可能积了多包
- `-ENODATA` 不是错误，表示 FIFO 已空
- 收到包后立刻更新 LED，直观显示通讯状态

---

## 5. TX 与 RX 的关键差异

| 项目 | TX (PTX) | RX (PRX) |
|------|----------|----------|
| `config.mode` | `ESB_MODE_PTX` | `ESB_MODE_PRX` |
| Radio 默认状态 | 空闲（IDLE） | **持续 RX** |
| 启动通讯 | `esb_write_payload()` | `esb_start_rx()` |
| `esb_write_payload` 含义 | 放入 TX FIFO，Radio 自动发送 | 放入 TX FIFO，作为 **ACK payload** |
| 主动发包 | ✅ | ❌（只能回 ACK） |
| 收到数据 | ACK payload（被动） | 空中数据包（主动监听） |
| 重传机制 | 自动重传（0~N 次可配） | 无（只管收） |
| `ready` 流控 | 需要 | 不需要 |
| 典型角色 | 鼠标/键盘 | Dongle/接收器 |

---

## 6. 从例程到产品：键鼠应用改造思路

### 6.1 鼠标端 (TX/PTX) 改造要点

**1. 从定时发送 → 事件驱动发送**

例程是 100ms 定时发，鼠标应该**有数据才发**：

```c
// 改造思路
while (1) {
    if (mouse_data_ready && esb_tx_ready) {
        esb_tx_ready = false;
        tx_payload.data[0] = mouse_report.buttons;
        tx_payload.data[1] = mouse_report.x & 0xFF;
        tx_payload.data[2] = mouse_report.x >> 8;
        // ... 组装报告
        esb_write_payload(&tx_payload);
    }
    k_sleep(K_USEC(125));   // 8K Hz 节拍
}
```

**2. esb_flush_tx() 的合理使用**

```c
// 位移数据：flush 旧数据，只发最新（coalescing）
esb_flush_tx();
esb_write_payload(&latest_displacement);

// 按键事件：不 flush，确保不丢
esb_write_payload(&key_event);
```

**3. noack 的策略性使用**

```c
// 位移帧：不要 ACK（低延迟，丢一帧无所谓）
tx_payload.noack = true;

// 按键帧：要 ACK（不能丢）
tx_payload.noack = false;
```

**4. 跳频 (Frequency Hopping)**

```c
static uint8_t hop_table[] = {5, 22, 45, 67, 89};  // 跳频表
static int hop_idx = 0;

// 每 N 包切换信道
if (pkt_count % 10 == 0) {
    esb_set_rf_channel(hop_table[hop_idx++ % ARRAY_SIZE(hop_table)]);
}
```

### 6.2 Dongle 端 (RX/PRX) 改造要点

**1. 多 pipe 对应多设备**

```c
// Pipe 0: 鼠标
// Pipe 1: 键盘
// Pipe 2: 数位板
```

每个 pipe 用不同 prefix，收到包时 `rx_payload.pipe` 告诉你来自哪个设备。

**2. 通过 ACK payload 下发配置**

```c
// Dongle 收到鼠标包 → 把配置写入 TX FIFO
// → 下次 ACK 时自动带过去
struct config_cmd cmd = { .dpi = 1600, .led = LED_BREATH };
esb_write_payload(&cmd);  // 放入 TX FIFO，自动附在 ACK 里
```

**3. USB HID 转发**

Dongle 收到 ESB 包 → 解析成 HID 报告 → 通过 USB HID 发送给 PC。
这部分需要结合 nRF54L20 的 USB HID 驱动（本工程不包含，可参考 `nRF54lm20_boot` 项目）。

### 6.3 常见坑位

| 坑 | 现象 | 原因 | 解决 |
|----|------|------|------|
| 多套设备串扰 | AB 鼠标交叉控制 | 地址相同 | 量产写入独立地址/配对协议 |
| TX FIFO 溢出 | 回调不触发 | 连续写多包不等 ready | 加 `ready` 流控 |
| 接收丢包 | 10 米外断续 | 信道干扰/功率不足 | 跳频 + 调 TX power |
| ACK payload 为空 | PRX 发了 PTX 收不到 | `esb_write_payload` 返回 -ENOMEM | FIFO 只存 3 包，调慢写速或加大 `CONFIG_ESB_TX_FIFO_SIZE` |
| 中断回调耗时 | 主循环卡死 | event_handler 里 sleep | 回调只记状态 + 投事件 |
| 射频和 BLE 冲突 | ESB 初始化失败 | 未配 MPSL timeslot | 参考 `nRF54lm20_boot` 项目开启 `CONFIG_ESB_MPSL_TIMESLOT` |

---

## 7. 编译与烧录

### 7.1 环境要求

- nRF Connect SDK v3.3.0（路径：`F:/Coding/ncs/v3.3.0`）
- Python 3.10+（路径：`C:/Users/stark/AppData/Local/Programs/Python/Python310`）
- Zephyr SDK 0.17.0（路径：`C:/ncs/toolchains/936afb6332/opt/zephyr-sdk`）

### 7.2 环境变量

```bash
export ZEPHYR_BASE="F:/Coding/ncs/v3.3.0/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR="C:/ncs/toolchains/936afb6332/opt/zephyr-sdk"
export PATH="/c/Users/stark/AppData/Local/Programs/Python/Python310:$PATH"
export PATH="/c/Users/stark/AppData/Local/Programs/Python/Python310/Scripts:$PATH"
export PATH="/c/ncs/toolchains/936afb6332/opt/zephyr-sdk/arm-zephyr-eabi/bin:$PATH"
export PATH="/c/ncs/toolchains/936afb6332/bin:$PATH"
```

### 7.3 编译

```bash
# TX
cd nRF54lm20_24G/tx
west build -b nrf54lm20dk/nrf54lm20a/cpuapp -d build

# RX
cd nRF54lm20_24G/rx
west build -b nrf54lm20dk/nrf54lm20a/cpuapp -d build
```

### 7.4 烧录

```bash
west flash -d build
# 或使用 nRF Connect for Desktop → Programmer
```

### 7.5 查看日志

- 用 **J-Link RTT Viewer** 连接 DK 的 SEGGER J-Link OB
- 或开启 UART 日志（需配 `prj.conf` + overlay）

### 7.6 资源占用

| 工程 | FLASH | RAM |
|------|-------|-----|
| TX | 53,420 B (2.69% of 1940KB) | 9,296 B (1.78% of 511KB) |
| RX | 54,456 B (2.74% of 1940KB) | 9,296 B (1.78% of 511KB) |

---

## 核心文件一览

| 文件 | 作用 |
|------|------|
| [tx/src/main.c](tx/src/main.c) | TX PTX 全部逻辑（初始化 + 事件回调 + 主循环） |
| [rx/src/main.c](rx/src/main.c) | RX PRX 全部逻辑 |
| [tx/prj.conf](tx/prj.conf) | Kconfig 配置（ESB 开关、DK LED 库等） |
| [tx/Kconfig](tx/Kconfig) | 应用级配置项（日志等级） |
| [tx/CMakeLists.txt](tx/CMakeLists.txt) | CMake 构建脚本 |
