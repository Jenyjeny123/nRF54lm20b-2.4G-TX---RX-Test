/*
 * nRF54LM20 2.4G Mouse TX — 状态机 + 8K DPPI + 10ms Window
 *
 * 状态: IDLE → PAIRING/RECONNECTING → CONNECTED
 * CONNECTED: DPPI 8K 数据 + 每 10ms 开 RX 窗口收 Dongle 指令
 *
 * 架构:
 *   DPPI 模式下 ISR 只处理 TX 数据（快速路径）
 *   控制模式下直接 poll radio 事件（不依赖 ISR）
 */
#include <stdio.h>
#include <string.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/devicetree.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_radio.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include "protocol.h"

LOG_MODULE_REGISTER(tx, LOG_LEVEL_INF);

/* ===== NVS 配对信息存储 ===== */
#define NVS_ID_PAIRED  1
#define NVS_ID_PEER    2
#define NVS_ID_MY      3

static struct nvs_fs nvs;
static uint8_t nvs_buf[64];  /* NVS 工作缓冲, 最小 3 * 页大小对齐 */

static int nvs_init(void) {
	nvs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if(!device_is_ready(nvs.flash_device)) return -ENODEV;
	nvs.offset = FIXED_PARTITION_OFFSET(storage_partition);
	nvs.sector_size = 4096; /* nRF flash page size */
	nvs.sector_count = FIXED_PARTITION_SIZE(storage_partition) / 4096;
	return nvs_mount(&nvs);
}
static void nvs_save_pairing(uint32_t peer, uint32_t my) {
	uint8_t flag=1;
	nvs_write(&nvs, NVS_ID_PAIRED, &flag, 1);
	nvs_write(&nvs, NVS_ID_PEER,   &peer, 4);
	nvs_write(&nvs, NVS_ID_MY,     &my,   4);
	printk("NVS: saved peer=0x%08X my=0x%08X\n", peer, my);
}
static bool nvs_load_pairing(uint32_t *peer, uint32_t *my) {
	uint8_t flag=0; int r;
	r=nvs_read(&nvs, NVS_ID_PAIRED, &flag, 1);
	if(r<=0 || flag!=1) return false;
	nvs_read(&nvs, NVS_ID_PEER, peer, 4);
	nvs_read(&nvs, NVS_ID_MY,   my,   4);
	printk("NVS: loaded peer=0x%08X my=0x%08X\n", *peer, *my);
	return true;
}
#define DP(...) do { printk(__VA_ARGS__); k_busy_wait(1000); } while(0)

/* ===== Radio 配置 ===== */
#define RADIO_PKT_MAX_LEN 40
#define HOP_CHANNELS_NUM  8
static const uint8_t hop_channels[HOP_CHANNELS_NUM]={2,14,36,56,66,78,0,22};
#define DPPI_CH_TXEN    2
#define DPPI_CH_DISABLE 3

/* ===== 全局状态 ===== */
static link_state_t link_state = ST_IDLE;
static bool         has_pairing_info;
static volatile bool btn0_pressed, btn1_down;
static int64_t      btn1_down_ms;
static bool         btn1_long;
static int64_t      state_entry_ms, last_window_ms, last_stat_ms;
static uint32_t     tx_total, tx_last_count;
static uint32_t     peer_addr, my_addr;
static uint8_t      radio_pkt[RADIO_PKT_MAX_LEN];
static uint8_t      rx_pkt[RADIO_PKT_MAX_LEN];
static uint8_t      current_channel;
static volatile bool test_running, dppi_running;

/* ===== 位翻转 ===== */
static uint32_t swap_bits(uint32_t inp) {
	uint32_t r=0; inp&=0xFF;
	for(int i=0;i<8;i++) r|=((inp>>i)&1)<<(7-i); return r;
}
static uint32_t bytewise_bitswap(uint32_t inp) {
	return (swap_bits(inp>>24)<<24)|(swap_bits(inp>>16)<<16)|(swap_bits(inp>>8)<<8)|swap_bits(inp);
}
static void radio_hop(void) {
	current_channel=(current_channel+1)%HOP_CHANNELS_NUM;
	*(volatile uint32_t*)((uint8_t*)NRF_RADIO+0x70C)&=~(1u<<31);
	nrf_radio_frequency_set(NRF_RADIO,2400+hop_channels[current_channel]);
	*(volatile uint32_t*)((uint8_t*)NRF_RADIO+0x07C)=1;
}
static void radio_set_addr(uint32_t b0, uint32_t b1, uint8_t p0) {
	NRF_RADIO->PREFIX0=((uint32_t)swap_bits(p0)<<0);
	NRF_RADIO->BASE0=bytewise_bitswap(b0); NRF_RADIO->BASE1=bytewise_bitswap(b1);
}
#define ADDR_PUBLIC 0x01234567u
#define ADDR_ALT    0x89ABCDEFu

/* ===== Radio 模式切换（轮询，不走 ISR）===== */
static void radio_tx_mode(void) {
	NRF_RADIO->SHORTS=RADIO_SHORTS_TXREADY_START_Msk|RADIO_SHORTS_PHYEND_DISABLE_Msk;
	NRF_RADIO->PACKETPTR=(uint32_t)radio_pkt;
	NRF_RADIO->TASKS_DISABLE=1; while(!NRF_RADIO->EVENTS_DISABLED){}
	NRF_RADIO->EVENTS_DISABLED=0;
}
static void radio_rx_mode(void) {
	NRF_RADIO->SHORTS=RADIO_SHORTS_RXREADY_START_Msk|RADIO_SHORTS_PHYEND_DISABLE_Msk
	                 |RADIO_SHORTS_DISABLED_RXEN_Msk|RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
	NRF_RADIO->PACKETPTR=(uint32_t)rx_pkt;
	NRF_RADIO->TASKS_DISABLE=1; while(!NRF_RADIO->EVENTS_DISABLED){}
	NRF_RADIO->EVENTS_DISABLED=0;
	NRF_RADIO->TASKS_RXEN=1;
}
/* 发一包并等 PHYEND（轮询） */
static void radio_tx_send(uint8_t len) {
	NRF_RADIO->PACKETPTR=(uint32_t)radio_pkt;
	NRF_RADIO->TASKS_TXEN=1; while(!NRF_RADIO->EVENTS_READY){}
	NRF_RADIO->EVENTS_READY=0; NRF_RADIO->TASKS_START=1;
	while(!NRF_RADIO->EVENTS_END){} NRF_RADIO->EVENTS_END=0;
}
/* 等 RX 收完(END)，返回 CRC 是否通过 */
static bool radio_rx_wait(uint32_t timeout_us) {
	NRF_RADIO->EVENTS_END=0;
	int64_t deadline=k_uptime_get()*1000+timeout_us;
	while(k_uptime_get()*1000<deadline && !NRF_RADIO->EVENTS_END){}
	if(NRF_RADIO->EVENTS_END) { NRF_RADIO->EVENTS_END=0; return NRF_RADIO->CRCSTATUS==1; }
	return false;
}

/* 发控制包并等 ACK（完全轮询） */
static bool send_and_wait_ack(uint8_t len, uint32_t timeout_us) {
	radio_tx_mode();
	radio_pkt[0]|=TYPE_MASK_ACK; radio_tx_send(len);
	radio_rx_mode();
	bool ok=radio_rx_wait(timeout_us);
	if(ok) ok=(rx_pkt[0]==TYPE_ACK);
	radio_tx_mode();
	return ok;
}

/* ===== ISR（仅 DPPI 数据模式，极快）===== */
ISR_DIRECT_DECLARE(radio_isr) {
	if(NRF_RADIO->EVENTS_READY) { NRF_RADIO->EVENTS_READY=0; }
	if(NRF_RADIO->EVENTS_END) {
		/* ★ 仅在 DPPI 数据模式下由 ISR 处理 END；
		 * 配对/回连/Window 模式由主循环轮询，ISR 不碰 */
		if(dppi_running && test_running && link_state==ST_CONNECTED) {
			NRF_RADIO->EVENTS_END=0;
			tx_total++; radio_hop();
			radio_pkt[0]=TYPE_MOUSE;
			radio_pkt[1]=(uint8_t)(tx_total&0xFF);
			radio_pkt[2]=(uint8_t)((tx_total>>8)&0xFF);
			radio_pkt[3]=(uint8_t)((tx_total>>16)&0xFF);
			radio_pkt[4]=(uint8_t)((tx_total>>24)&0xFF);
		}
	}
	if(NRF_RADIO->EVENTS_DISABLED) { NRF_RADIO->EVENTS_DISABLED=0; }
	if(NRF_RADIO->EVENTS_RXREADY)  { NRF_RADIO->EVENTS_RXREADY=0; }
	return 0;
}

/* ===== Radio 初始化 ===== */
static void radio_init(void) {
	DP("R1m "); nrf_radio_mode_set(NRF_RADIO,NRF_RADIO_MODE_NRF_4MBIT_BT_0_4);
	current_channel=0; nrf_radio_frequency_set(NRF_RADIO,2400+hop_channels[0]);
	nrf_radio_txpower_set(NRF_RADIO,NRF_RADIO_TXPOWER_POS8DBM);
	radio_set_addr(ADDR_PUBLIC,ADDR_ALT,0xC0);
	NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1;
	DP("R2 "); NRF_RADIO->PCNF0=(0<<RADIO_PCNF0_S1LEN_Pos)|(1<<RADIO_PCNF0_S0LEN_Pos)|(1<<RADIO_PCNF0_LFLEN_Pos);
	NRF_RADIO->PCNF1=(RADIO_PCNF1_WHITEEN_Disabled<<RADIO_PCNF1_WHITEEN_Pos)|(RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos)|(4<<RADIO_PCNF1_BALEN_Pos)|(RADIO_PKT_MAX_LEN<<RADIO_PCNF1_STATLEN_Pos)|(RADIO_PKT_MAX_LEN<<RADIO_PCNF1_MAXLEN_Pos);
	DP("R3 "); NRF_RADIO->CRCCNF=(RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos); NRF_RADIO->CRCINIT=0xFFFFUL; NRF_RADIO->CRCPOLY=0x11021UL;
	DP("R4 "); NRF_RADIO->INTENSET00=RADIO_INTENSET00_READY_Msk|RADIO_INTENSET00_END_Msk|RADIO_INTENSET00_DISABLED_Msk|RADIO_INTENSET00_RXREADY_Msk;
	IRQ_DIRECT_CONNECT(RADIO_0_IRQn,0,radio_isr,0); NVIC_ClearPendingIRQ(RADIO_0_IRQn); irq_enable(RADIO_0_IRQn);
	DP("R5 "); nrf_radio_fast_ramp_up_enable_set(NRF_RADIO,true);
	radio_tx_mode(); DP("done\n");
}

/* ===== DPPI ===== */
static void dppi_init(void) {
	DP("D1 "); NRF_TIMER10->TASKS_CLEAR=1; NRF_TIMER10->MODE=0; NRF_TIMER10->BITMODE=3; NRF_TIMER10->PRESCALER=5;
	NRF_TIMER10->CC[0]=125; NRF_TIMER10->SHORTS=TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	NRF_TIMER10->PUBLISH_COMPARE[0]=((1u<<31)|DPPI_CH_TXEN);
	NRF_RADIO->SUBSCRIBE_TXEN=((1u<<31)|DPPI_CH_TXEN);
	NRF_DPPIC10->CHEN=(1<<DPPI_CH_TXEN)|(1<<DPPI_CH_DISABLE); DP("done\n");
}
static void dppi_start(void) { dppi_running=true; NRF_TIMER10->TASKS_START=1; }
static void dppi_stop(void)  { dppi_running=false; NRF_TIMER10->TASKS_STOP=1; }

/* ===== 按钮 ===== */
static void btn_handler(uint32_t pressed, uint32_t changed) {
	if(pressed & changed & DK_BTN1_MSK) btn0_pressed=true;
	if(pressed & changed & DK_BTN2_MSK) { btn1_down=true; btn1_down_ms=k_uptime_get(); }
	if((!pressed) & changed & DK_BTN2_MSK) { btn1_down=false; btn1_long=false; }
}

/* ===== LED ===== */
static uint32_t led_ms; static bool led_on;
static void led_blink(uint32_t period) {
	if(k_uptime_get()-led_ms>=period/2){ led_ms=k_uptime_get(); led_on=!led_on; dk_set_led(DK_LED1,led_on?1:0); }
}
static void led_update(void) {
	switch(link_state){
	case ST_PAIRING:      led_blink(200); break;
	case ST_RECONNECTING: led_blink(1000); break;
	case ST_CONNECTED: dk_set_led(DK_LED1,(k_uptime_get()-state_entry_ms<1000)?1:0); break;
	default: dk_set_led(DK_LED1,0); break;
	}
}

/* ===== 配对/回连 ===== */
static void pairing_tick(void) {
	static int64_t last_req;
	if(k_uptime_get()-last_req<50) return; last_req=k_uptime_get();
	radio_pkt[0]=TYPE_PAIR_REQ; *(uint32_t*)&radio_pkt[1]=my_addr;
	if(send_and_wait_ack(6, ACK_TIMEOUT_US)) {
		peer_addr=*(uint32_t*)&rx_pkt[1];
		radio_pkt[0]=TYPE_PAIR_CONFIRM;
		if(send_and_wait_ack(2, ACK_TIMEOUT_US)) {
			has_pairing_info=true;
			nvs_save_pairing(peer_addr, my_addr);
			printk("PAIR OK! peer=0x%08X\n",peer_addr);
			link_state=ST_CONNECTED; state_entry_ms=k_uptime_get();
		}
	}
}
static void reconnect_tick(void) {
	static int64_t last_req;
	if(k_uptime_get()-last_req<50) return; last_req=k_uptime_get();
	radio_pkt[0]=TYPE_RECONN_REQ; *(uint32_t*)&radio_pkt[1]=peer_addr;
	if(send_and_wait_ack(6, ACK_TIMEOUT_US)) {
		printk("RECONN OK!\n");
		link_state=ST_CONNECTED; state_entry_ms=k_uptime_get();
	}
}

/* ===== Window ===== */
static void window_open(void) {
	if(!dppi_running) return;
	dppi_stop();
	radio_tx_mode();
	radio_pkt[0]=TYPE_WINDOW_OPEN; *(uint16_t*)&radio_pkt[1]=(uint16_t)WINDOW_DURATION_US;
	radio_tx_send(4);
	radio_rx_mode();
	bool ok=radio_rx_wait(WINDOW_DURATION_US);
	if(ok && rx_pkt[0]==TYPE_CMD_DONGLE) {
		uint8_t cmd=rx_pkt[1]; printk("CMD:0x%02X\n",cmd);
		radio_tx_mode();
		radio_pkt[0]=TYPE_ACK; radio_tx_send(2);
	}
	radio_tx_mode(); dppi_start();
}

/* ===== HFCLK ===== */
static int clk_start(void) {
	const struct device *c=DEVICE_DT_GET_OR_NULL(DT_NODELABEL(clock));
	if(!c||!device_is_ready(c)) return -ENODEV;
	return clock_control_on(c,(clock_control_subsys_t)CLOCK_CONTROL_NRF_SUBSYS_HF);
}

/* ================================================================ */
int main(void) {
	printk("\n=== nRF54LM20 Mouse TX ===\n");
	DP("CLK "); if(clk_start()){printk("CLK FAIL\n");return 0;} DP("ok\n");
	radio_init(); dppi_init();
	dk_leds_init(); dk_buttons_init(btn_handler);

	/* NVS 初始化 + 读配对信息 */
	if(nvs_init()) { printk("NVS init FAIL\n"); has_pairing_info=false; my_addr=0x12345678u; }
	else has_pairing_info=nvs_load_pairing(&peer_addr, &my_addr);
	if(!has_pairing_info) { my_addr=0x12345678u; peer_addr=0; }

	if(has_pairing_info) { link_state=ST_RECONNECTING; radio_set_addr(peer_addr,ADDR_ALT,0xC0); }
	else                 { link_state=ST_PAIRING;      radio_set_addr(ADDR_PUBLIC,ADDR_ALT,0xC0); }
	radio_tx_mode(); state_entry_ms=k_uptime_get();
	printk("Init done. state=%d has_pair=%d\n",link_state,has_pairing_info);

	while(1) {
		/* Btn1 长按 → 配对 */
		if(btn1_down && !btn1_long && k_uptime_get()-btn1_down_ms>LONG_PRESS_MS) {
			btn1_long=true;
			if(link_state==ST_RECONNECTING) {
				printk("Long press → PAIRING\n");
				link_state=ST_PAIRING; state_entry_ms=k_uptime_get();
				radio_set_addr(ADDR_PUBLIC,ADDR_ALT,0xC0); radio_tx_mode();
			}
		}
		/* Btn0 测试开关 */
		static int64_t cool=-2000;
		if(btn0_pressed && k_uptime_get()-cool>1000) {
			btn0_pressed=false; cool=k_uptime_get();
			if(link_state==ST_CONNECTED) {
				if(!test_running) {
					printk(">>> TEST START\n"); tx_total=tx_last_count=0;
					last_stat_ms=k_uptime_get(); test_running=true; dppi_start();
				} else {
					dppi_stop(); test_running=false; printk("<<< TEST STOP\n");
				}
			}
		}
		/* 状态机 */
		switch(link_state) {
		case ST_PAIRING:
			pairing_tick();
			if(k_uptime_get()-state_entry_ms>PAIR_TIMEOUT_S*1000) {
				printk("Pair timeout\n"); k_sleep(K_SECONDS(1)); NVIC_SystemReset();
			}
			break;
		case ST_RECONNECTING:
			reconnect_tick();
			if(k_uptime_get()-state_entry_ms>RECONN_TIMEOUT_S*1000) {
				printk("Reconn timeout\n"); k_sleep(K_SECONDS(1)); NVIC_SystemReset();
			}
			break;
		case ST_CONNECTED:
			if(test_running && k_uptime_get()-last_window_ms>=10) {
				last_window_ms=k_uptime_get(); window_open();
			}
			if(test_running && k_uptime_get()-last_stat_ms>=1000) {
				uint32_t pps=tx_total-tx_last_count;
				printk("[TX] +%u pkt/s | total:%u\n",pps,tx_total);
				tx_last_count=tx_total; last_stat_ms=k_uptime_get();
			}
			break;
		default: break;
		}
		led_update();
		if(link_state!=ST_CONNECTED) k_sleep(K_MSEC(10));
	}
}
