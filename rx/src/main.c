/*
 * nRF54LM20 2.4G Dongle RX — 状态机 + 连续接收 + Window 处理
 *
 * 状态: 上电前2s→PAIRING, 2s后→RECONNECTING, 成功→CONNECTED
 * CONNECTED: 持续 8K 收数据 + 收到 WINDOW_OPEN 后发 CMD
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

LOG_MODULE_REGISTER(rx, LOG_LEVEL_INF);

/* ===== NVS 配对信息存储 ===== */
#define NVS_ID_PAIRED  1
#define NVS_ID_PEER    2
#define NVS_ID_MY      3

static struct nvs_fs nvs;
static uint8_t nvs_buf[64];

static int nvs_init(void) {
	
	nvs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if(!device_is_ready(nvs.flash_device)) return -ENODEV;
	nvs.offset = FIXED_PARTITION_OFFSET(storage_partition);
	
	
	nvs.sector_size = 4096;
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
#define SCAN_TIMER NRF_TIMER10

/* ===== 全局状态 ===== */
static link_state_t link_state = ST_IDLE;
static bool         has_pairing_info;
static int64_t      state_entry_ms, last_stat_ms;
static volatile bool rx_synchronized;
static volatile uint32_t rx_total, rx_lost, rx_dup, rx_last_seq;
static volatile bool     rx_seq_init;
static volatile uint32_t rx_rssi_sum, rx_rssi_cnt;
static volatile bool     ctrl_mode;        /* 控制事务中，ISR 不碰 END */
static volatile bool     ev_got_ctrl;
static volatile uint8_t  ev_ctrl_type;
static uint32_t     peer_addr, my_addr;
static uint8_t      radio_pkt[RADIO_PKT_MAX_LEN];
static uint8_t      rx_pkt[RADIO_PKT_MAX_LEN];
static uint8_t      current_channel;
static uint32_t     rx_last_count;
static bool         test_active;
static volatile bool rx_ok;

/* ===== 位翻转/跳频 ===== */
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

/* ===== Radio 模式 ===== */
static void radio_rx_mode(void) {
	NRF_RADIO->SHORTS=RADIO_SHORTS_RXREADY_START_Msk|RADIO_SHORTS_PHYEND_DISABLE_Msk
	                 |RADIO_SHORTS_DISABLED_RXEN_Msk|RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
	NRF_RADIO->PACKETPTR=(uint32_t)rx_pkt;
	NRF_RADIO->TASKS_DISABLE=1; while(!NRF_RADIO->EVENTS_DISABLED){}
	NRF_RADIO->EVENTS_DISABLED=0;
	NRF_RADIO->TASKS_RXEN=1;
}
static void radio_tx_mode(void) {
	NRF_RADIO->SHORTS=RADIO_SHORTS_TXREADY_START_Msk|RADIO_SHORTS_PHYEND_DISABLE_Msk;
	NRF_RADIO->PACKETPTR=(uint32_t)radio_pkt;
	NRF_RADIO->TASKS_DISABLE=1; while(!NRF_RADIO->EVENTS_DISABLED){}
	NRF_RADIO->EVENTS_DISABLED=0;
}
static void radio_tx_send(uint8_t len) {
	NRF_RADIO->PACKETPTR=(uint32_t)radio_pkt;
	NRF_RADIO->TASKS_TXEN=1; while(!NRF_RADIO->EVENTS_READY){}
	NRF_RADIO->EVENTS_READY=0; NRF_RADIO->TASKS_START=1;
	while(!NRF_RADIO->EVENTS_END){} NRF_RADIO->EVENTS_END=0;
}

/* ===== ISR ===== */
ISR_DIRECT_DECLARE(radio_isr) {
	if(NRF_RADIO->EVENTS_READY)  { NRF_RADIO->EVENTS_READY=0; rx_ok=false; }
	if(NRF_RADIO->EVENTS_END) {
		if(ctrl_mode) return 0;  /* 控制事务中 → 不处理，留给主循环轮询 */
		NRF_RADIO->EVENTS_END=0;
		if(NRF_RADIO->CRCSTATUS==1) {
			rx_ok=true;
			uint32_t rssi=NRF_RADIO->RSSISAMPLE; rx_rssi_sum+=rssi; rx_rssi_cnt++;
			uint8_t t=rx_pkt[0];
			if(t & TYPE_MASK_ACK) {
				/* 控制包 → 通知 main 处理 */
				ev_got_ctrl=true; ev_ctrl_type=t;
			} else if(test_active) {
				/* 数据包 → 统计 */
				rx_total++;
				uint32_t seq=rx_pkt[1]|((uint32_t)rx_pkt[2]<<8)|((uint32_t)rx_pkt[3]<<16)|((uint32_t)rx_pkt[4]<<24);
				if(!rx_seq_init){ rx_seq_init=true; rx_last_seq=seq; }
				else if(seq==rx_last_seq) rx_dup++;
				else if(seq>rx_last_seq){ rx_lost+=seq-rx_last_seq-1; rx_last_seq=seq; }
				else { rx_lost+=(0xFFFFFFFFu-rx_last_seq)+seq; rx_last_seq=seq; }
			}
		}
	}
	if(NRF_RADIO->EVENTS_DISABLED) { NRF_RADIO->EVENTS_DISABLED=0; if(rx_ok) radio_hop(); }
	if(NRF_RADIO->EVENTS_RXREADY)  { NRF_RADIO->EVENTS_RXREADY=0; }
	return 0;
}

/* ===== Radio 初始化 ===== */
static void radio_init(void) {
	DP("R1 "); nrf_radio_mode_set(NRF_RADIO,NRF_RADIO_MODE_NRF_4MBIT_BT_0_4);
	current_channel=0; nrf_radio_frequency_set(NRF_RADIO,2400+hop_channels[0]);
	nrf_radio_txpower_set(NRF_RADIO,NRF_RADIO_TXPOWER_POS8DBM);
	radio_set_addr(ADDR_PUBLIC,ADDR_ALT,0xC0);
	NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1;
	DP("R2 "); NRF_RADIO->PCNF0=(0<<RADIO_PCNF0_S1LEN_Pos)|(1<<RADIO_PCNF0_S0LEN_Pos)|(1<<RADIO_PCNF0_LFLEN_Pos);
	NRF_RADIO->PCNF1=(RADIO_PCNF1_WHITEEN_Disabled<<RADIO_PCNF1_WHITEEN_Pos)|(RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos)|(4<<RADIO_PCNF1_BALEN_Pos)|(RADIO_PKT_MAX_LEN<<RADIO_PCNF1_STATLEN_Pos)|(RADIO_PKT_MAX_LEN<<RADIO_PCNF1_MAXLEN_Pos);
	DP("R3 "); NRF_RADIO->CRCCNF=(RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos); NRF_RADIO->CRCINIT=0xFFFFUL; NRF_RADIO->CRCPOLY=0x11021UL;
	DP("R4 "); NRF_RADIO->INTENSET00=RADIO_INTENSET00_READY_Msk|RADIO_INTENSET00_END_Msk|RADIO_INTENSET00_DISABLED_Msk|RADIO_INTENSET00_RXREADY_Msk;
	IRQ_DIRECT_CONNECT(RADIO_0_IRQn,0,radio_isr,0); NVIC_ClearPendingIRQ(RADIO_0_IRQn); irq_enable(RADIO_0_IRQn);
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO,true);
	radio_rx_mode(); DP("done\n");
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

/* ===== 控制包处理（收到 REQ → 切 TX → 回 RESP → 切回 RX）===== */
static void handle_pair_req(void) {
	ctrl_mode=true;
	peer_addr=*(uint32_t*)&rx_pkt[1];
	radio_tx_mode();
	radio_pkt[0]=TYPE_PAIR_RESP; *(uint32_t*)&radio_pkt[1]=my_addr;
	radio_tx_send(6);
	radio_rx_mode();
	int64_t t=k_uptime_get()*1000+1000;
	while(k_uptime_get()*1000<t && !ev_got_ctrl){}
	if(ev_got_ctrl && ev_ctrl_type==TYPE_PAIR_CONFIRM) {
		has_pairing_info=true;
		nvs_save_pairing(peer_addr, my_addr);
		printk("PAIR OK! peer=0x%08X\n",peer_addr);
		link_state=ST_CONNECTED; state_entry_ms=k_uptime_get(); test_active=true;
		rx_total=rx_lost=rx_dup=0; rx_seq_init=false; rx_last_count=0; last_stat_ms=k_uptime_get();
	}
	ev_got_ctrl=false;
	ctrl_mode=false;
	radio_rx_mode();
}
static void handle_reconn_req(void) {
	uint32_t req_addr=*(uint32_t*)&rx_pkt[1];
	if(req_addr==my_addr || link_state==ST_PAIRING) {
		ctrl_mode=true;
		radio_tx_mode();
		radio_pkt[0]=TYPE_RECONN_RESP; radio_tx_send(2);
		radio_rx_mode();
		ctrl_mode=false;
		printk("RECONN OK!\n");
		link_state=ST_CONNECTED; state_entry_ms=k_uptime_get(); test_active=true;
		rx_total=rx_lost=rx_dup=0; rx_seq_init=false; rx_last_count=0; last_stat_ms=k_uptime_get();
	}
}
static void handle_window(void) {
	ctrl_mode=true;
	radio_tx_mode();
	radio_pkt[0]=TYPE_CMD_DONGLE; radio_pkt[1]=CMD_GET_BATTERY;
	radio_tx_send(2);
	int64_t t=k_uptime_get()*1000+500;
	radio_rx_mode();
	while(k_uptime_get()*1000<t && !ev_got_ctrl){}
	if(ev_got_ctrl && ev_ctrl_type==TYPE_ACK) {}
	ev_got_ctrl=false;
	ctrl_mode=false;
	radio_rx_mode();
}

/* ===== HFCLK ===== */
static int clk_start(void) {
	const struct device *c=DEVICE_DT_GET_OR_NULL(DT_NODELABEL(clock));
	if(!c||!device_is_ready(c)) return -ENODEV;
	return clock_control_on(c,(clock_control_subsys_t)CLOCK_CONTROL_NRF_SUBSYS_HF);
}

/* ================================================================ */
int main(void) {
	printk("\n=== nRF54LM20 Dongle RX ===\n");
	DP("CLK "); if(clk_start()){printk("CLK FAIL\n");return 0;} DP("ok\n");
	radio_init();
	dk_leds_init();
	/* NVS 初始化 + 读配对信息 */
	if(nvs_init()) { printk("NVS init FAIL\n"); has_pairing_info=false; my_addr=0x87654321u; }
	else has_pairing_info=nvs_load_pairing(&peer_addr, &my_addr);
	if(!has_pairing_info) { my_addr=0x87654321u; peer_addr=0; }

	/* 初始：配对模式（前 2s） */
	link_state=ST_PAIRING; state_entry_ms=k_uptime_get();
	radio_set_addr(ADDR_PUBLIC,ADDR_ALT,0xC0); radio_rx_mode();
	printk("Init done. PAIRING mode (2s window)\n");

	while(1) {
		/* 2s 后切换到回连 */
		if(link_state==ST_PAIRING && k_uptime_get()-state_entry_ms>2000) {
			if(has_pairing_info) {
				link_state=ST_RECONNECTING; state_entry_ms=k_uptime_get();
				radio_set_addr(my_addr,ADDR_ALT,0xC0); radio_rx_mode();
				printk("Switching to RECONNECTING\n");
			} else {
				/* 未配对过，保持在 PAIRING 但只收 RECONN_REQ(忽略), 只收 PAIR_REQ */
				/* 实际上就是没配对过就一直配对模式 */
				printk("No pair info, staying PAIRING\n");
				state_entry_ms=k_uptime_get(); /* 重置计时 */
			}
		}

		/* 处理收到的控制包 */
		if(ev_got_ctrl) {
			uint8_t t=ev_ctrl_type; ev_got_ctrl=false;
			if(link_state==ST_PAIRING && t==TYPE_PAIR_REQ) handle_pair_req();
			else if(t==TYPE_RECONN_REQ) handle_reconn_req();
			else if(link_state==ST_CONNECTED && t==TYPE_WINDOW_OPEN) handle_window();
		}

		/* 失同步检测 */
		static uint8_t miss;
		if(test_active && link_state==ST_CONNECTED) {
			if(!rx_ok) { miss++; if(miss>10) {
				printk("<<< Signal lost\n"); test_active=false; miss=0;
				link_state=ST_RECONNECTING; state_entry_ms=k_uptime_get();
				radio_set_addr(has_pairing_info?my_addr:ADDR_PUBLIC,ADDR_ALT,0xC0); radio_rx_mode();
			}}
			else miss=0;
		}

		/* 每秒统计 */
		if(test_active && k_uptime_get()-last_stat_ms>=1000) {
			uint32_t total=rx_total, lost=rx_lost, dup=rx_dup, cnt=rx_rssi_cnt;
			int32_t rssi=cnt>0?-(int32_t)(rx_rssi_sum/cnt):0;
			uint32_t pps=total-rx_last_count;
			printk("[RX] +%u pkt/s | ok:%u lost:%u dup:%u rssi:%ddBm",pps,total,lost,dup,rssi);
			if(total+lost>0){ uint32_t pm=lost*1000u/(total+lost); printk(" (loss %u.%u%%)",pm/10,pm%10); }
			printk("\n");
			rx_last_count=total; last_stat_ms=k_uptime_get(); rx_rssi_sum=0; rx_rssi_cnt=0;
		}

		led_update();
		k_sleep(K_MSEC(50));
	}
}
