# 2.4G Mouse-Dongle 协议设计文档

## 1. 产品定位

TX = 鼠标（nRF54LM20），RX = Dongle（nRF54LM20）。
- 鼠标 2.4G 报告率：8K Hz（8000 pps）
- 支持 PC 端 APP/驱动通过 Dongle 下发指令（改 DPI、灯效、固件升级等）
- 支持配对 / 回连 / 连接状态机

## 2. 状态机

### TX (Mouse)

```
上电 → 读Flash

情况A: 无配对信息
  配对模式 → 配对成功 → 连接
  配对模式 → 30s超时 → 复位

情况B: 有配对信息
  回连模式 → 回连成功 → 连接
  回连模式 → 30s超时 → 复位
  回连模式 → 长按3s(Btn1) → 配对模式 → 配对成功 → 连接(覆盖旧配对)
                            配对模式 → 30s超时 → 复位

连接后: 8K发数据 + 定期开窗收Dongle指令
```

### RX (Dongle)

```
上电

前2s: 配对模式
  收到PAIR_REQ → 配对成功 → 连接
  收到RECONN_REQ → 回连成功 → 连接

2s后: 回连模式
  收到RECONN_REQ → 回连成功 → 连接
  收到PAIR_REQ → 忽略

连接后: 持续收数据 + 窗口发CMD指令
```

- RX 只有一个 LED0 做状态指示

### LED 指示 (TX)

| 状态 | LED0 |
|------|------|
| 配对中 | 快闪（200ms 周期） |
| 回连中 | 慢闪（1s 周期） |
| 连接上 | 常亮 1s → 熄灭 |

### 按键 (TX)

| 按键 | 功能 |
|------|------|
| Btn0 | 测试开关（保留，开发用） |
| Btn1 | 回连模式下长按 3s 进入配对模式；配对模式下忽略 |

### 超时策略

| 场景 | 超时 | 行为 |
|------|------|------|
| 配对模式无结果 | 30s | 复位（后期改二级休眠） |
| 回连模式无结果 | 30s | 复位（后期改二级休眠） |
| 控制包等 ACK | 500μs | 重发，最多 10 次 |

## 3. 协议设计

### 空中帧格式

```
Radio 硬件自动处理:
  [S0(1B)] [LEN(1B)] [S1(1B)] [PAYLOAD] [CRC(2B)]
           ↑ LFLEN=1 → 动态长度

PAYLOAD:
  [0] TYPE (1B)
  [1..LEN-1] DATA

TYPE:
  bit7 = 0 : 不需要 ACK（高速单向）
  bit7 = 1 : 需要 ACK（双向可靠）
```

### 数据类（no ACK, 8K 高速）

| TYPE | 名称 | LEN | 内容 |
|------|------|-----|------|
| `0x00` | MOUSE | 8 | [1]btns [2]X_L [3]X_H [4]Y_L [5]Y_H [6]wheel [7]pan |
| `0x01` | KEYBOARD | 8 | [1]mod [2]rsv [3~7]key×5 |
| `0x02` | CONSUMER | 4 | [1~2]usage |
| `0x03` | SYSTEM | 3 | [1]key |
| `0x04~0x0F` | 预留 | — | — |

### 控制类（需 ACK, bit7=1）

| TYPE | 名称 | 方向 | LEN | 内容 |
|------|------|------|-----|------|
| `0x80` | WINDOW_OPEN | TX→RX | 4 | [1~2]窗口μs |
| `0x81` | CMD_DONGLE | RX→TX | ≤38 | [1]cmd_id [2~]参数 |
| `0x82` | ACK | TX→RX | 3 | [1]应答的seq |
| `0x83` | PAIR_REQ | TX→RX | 6 | [1~4]TX的随机addr |
| `0x84` | PAIR_RESP | RX→TX | 6 | [1~4]RX分配的addr |
| `0x85` | PAIR_CONFIRM | TX→RX | 2 | 空 |
| `0x86` | RECONN_REQ | TX→RX | 6 | [1~4]已保存的RX addr |
| `0x87` | RECONN_RESP | RX→TX | 2 | 空 |
| `0x88` | RSSI_REPORT | RX→TX | 3 | [1]rssi |
| `0x89` | BATTERY_REQ | RX→TX | 2 | 空 |
| `0x8A` | BATTERY_RESP | TX→RX | 3 | [1]电量% |
| `0x8B` | FW_VERSION_REQ | RX→TX | 2 | 空 |
| `0x8C` | FW_VERSION_RESP | TX→RX | 5 | [1]major [2]minor [3]patch |
| `0x8D` | DPI_CHANGE | RX→TX | 4 | [1~2]dpi |
| `0x8E` | LED_CONFIG | RX→TX | 6 | [1]mode [2]R [3]G [4]B [5]speed |
| `0x8F` | SLEEP_CMD | RX→TX | 2 | 空 |
| `0x90` | PAIR_BTN_PRESS | TX→RX | 2 | 通知RX进入配对（后期用） |
| `0x91~0xBF` | 预留 | — | — | — |

### CMD_DONGLE (0x81) 的 cmd_id

| cmd_id | 名称 | 参数 |
|--------|------|------|
| `0x01` | SET_DPI | [2]dpi_L [3]dpi_H |
| `0x02` | SET_LED | [2]mode [3]R [4]G [5]B [6]speed |
| `0x03` | SET_POLLING | [2]rate |
| `0x04` | GET_BATTERY | 空 |
| `0x05` | GET_FW_VERSION | 空 |
| `0x06` | SLEEP | 空 |
| `0x07` | FW_UPGRADE_START | 参数 |
| `0x08` | FW_DATA | 数据块 |
| `0x09` | RESET | 空 |
| `0x0A~0x3F` | 预留 | — |

## 4. 双向通讯机制：TDMA 时间分片

### 原理

Mouse 99% 时间 8K 单向发包（no ACK）。每 10ms 开一次 RX 窗口，让 Dongle 发指令。

```
时间轴（125μs 时隙）:
Mouse:  [...TX][TX][TX]...[TX][--RX窗口--][TX][TX]...
         ← 连发 79 包 →  ← 3~4时隙 → ← 恢复 →
         
Dongle: [...RX][RX][RX]...[RX][--发CMD,等ACK--][RX]...
         ← 持续收 →  ← 看到WINDOW后切TX → ← 恢复 →
```

### 窗口时序

```
1. Mouse 发 WINDOW_OPEN → 停 DPPI → DISABLE → 切 RXEN → 等
2. Dongle 收 WINDOW_OPEN → DISABLE → 切 TXEN → 发 CMD → 等 ACK
3. Mouse 收 CMD → 处理 → DISABLE → 切 TXEN → 发 ACK
4. Dongle 收 ACK → DISABLE → 切 RXEN → 恢复
5. Mouse 发完 ACK → DISABLE → 切 TXEN → 恢复 DPPI

窗口约 350~500μs（3~4 个时隙），丢 3~4 包，每 10ms 丢 0.04%
```

### 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 窗口间隔 | 10ms | 对齐 APP 发包频率 |
| 窗口时长 | ~500μs | 够发 38 字节 APP 数据 |
| 丢包/窗口 | 3~4 包 | 可忽略 |
| 控制包 ACK 超时 | 500μs | |
| 控制包最大重试 | 10 次 | |

## 5. 配对数据交换流程

### 地址方案

```
出厂：TX/RX 共用一套公共配对地址（所有设备相同）
配对后：各自生成随机 4B BASE 地址，通过配对协议交换
下次回连：直接用 Flash 中保存的配对地址

Radio 地址构成:
  PREFIX(1B) + BASE0(4B) = 5B 空中地址
```

### 配对时序

双方用公共配对地址。三步握手：

```
1. TX→RX: PAIR_REQ     (TX把自己的新地址发给RX)
2. RX→TX: PAIR_RESP    (RX把自己的新地址发给TX)
3. TX→RX: PAIR_CONFIRM (确认收到)
```

每步等 ACK 超时 500μs，TX 最多重试 10 次。完成后双方保存地址到 Flash，进入连接。

### 回连时序

双方用已保存的配对地址。两步握手：

```
1. TX→RX: RECONN_REQ (带已保存的RX地址)
2. RX→TX: RECONN_RESP (地址匹配则应答)
```

每步等 ACK 超时 500μs，TX 最多重试 3 次。成功后进入连接。

### 超时与重试

| 步骤 | 等 ACK 超时 | 最多重试 |
|------|-----------|---------|
| PAIR_REQ → PAIR_RESP | 500μs | 10 次 |
| PAIR_RESP → PAIR_CONFIRM | 500μs | 等对方重发 REQ |
| PAIR_CONFIRM | 500μs | 3 次 |
| RECONN_REQ → RECONN_RESP | 500μs | 3 次 |
| 配对总超时 | 30s | — |
| 回连总超时 | 30s | — |

### Flash 存储 (NVS)

| 字段 | 大小 | 说明 |
|------|------|------|
| paired_flag | 1B | 0=未配对, 1=已配对 |
| peer_addr | 4B | 对方的 Radio BASE 地址 |
| my_addr | 4B | 自己的 Radio BASE 地址 |

## 6. Radio 配置

| 参数 | 值 |
|------|-----|
| 模式 | NRF_RADIO_MODE_NRF_4MBIT_BT_0_4 |
| 速率 | 4 Mbps |
| 地址位宽 | 4 字节 (BALEN=4) |
| CRC | CRC16 |
| 动态长度 | LFLEN=1（1 字节长度头） |
| Fast ramp-up | 启用 |
| TX 功率 | +8 dBm |
| 跳频 | 8 信道，TX 每包一跳，RX 同步跳 |
| 公共配对地址 | BASE0=0x01234567, PREFIX0=0xC0 |

### SHORTS 配置

```
TX 高速模式: TXREADY→START | PHYEND→DISABLE
TX 控制模式(等ACK): 清 SHORTS, 手动切换
RX 正常模式: RXREADY→START | PHYEND→DISABLE | DISABLED→RXEN | ADDRESS→RSSISTART
RX 控制模式(回ACK): 清 SHORTS, 手动切 TX → 发 ACK → 恢复
```

## 7. 待实现步骤

### Phase 1: 协议层
- [ ] LFLEN=1 动态长度
- [ ] TYPE 表定义（头文件）
- [ ] 收发切换函数（radio_switch_to_tx / radio_switch_to_rx）
- [ ] Window 窗口机制（WINDOW_OPEN + 停/启 DPPI）

### Phase 2: 状态机
- [ ] TX 配对/回连/连接 状态机
- [ ] RX 配对/回连/连接 状态机
- [ ] 按键处理（Btn1 长按 3s）
- [ ] LED 指示

### Phase 3: 数据路径
- [ ] 8K 鼠标数据 + 定期开窗
- [ ] Dongle 收数据 + 窗口发指令
- [ ] NVS 存储配对信息

### Phase 4: APP 双向通讯
- [ ] CMD_DONGLE 指令处理和响应
- [ ] 电池/固件/DPI/LED 指令实现

## 8. 硬件资源

| 外设 | TX 用途 | RX 用途 |
|------|---------|---------|
| RADIO | 4Mbps 收发 | 同 |
| TIMER10 | DPPI 8K 触发 | 扫描定时器 |
| DPPIC10/20 | TIMER→RADIO 链 | — |
| GPIOTE20 | DPPI 引脚指示 | — |
| GPIOTE (普通) | Btn0, Btn1, LED0 | LED0 |
| NVS (Flash) | 配对信息存储 | 配对信息存储 |
