/*
 * nRF54LM20 2.4G 吞吐量测试 —— RX (裸 Radio 连续接收)
 * 基于 LRchangyu/nrf54l_radio 移植
 *
 * V2 优化：ISR 去 scan_timer_start，丢包检测用 cycle counter
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

LOG_MODULE_REGISTER(rx_radio, LOG_LEVEL_INF);

#define DP(...) do { printk(__VA_ARGS__); k_busy_wait(1000); } while(0)

#define RADIO_PKT_LEN    8
#define HOP_CHANNELS_NUM 8
static const uint8_t hop_channels[HOP_CHANNELS_NUM] = {2,14,36,56,66,78,0,22};

#define SCAN_TIMER       NRF_TIMER10
#define SCAN_TIMER_IRQ   TIMER10_IRQHandler
#define SCAN_TIMER_IRQn  TIMER10_IRQn

static volatile bool rx_synchronized, rx_channel_timeout, rx_ok;
static volatile uint32_t rx_total, rx_lost, rx_dup, rx_last_seq;
static volatile bool     rx_seq_init;
static volatile uint32_t rx_rssi_sum, rx_rssi_cnt;  /* RSSI 累加 */
static uint8_t  radio_packet[RADIO_PKT_LEN];
static uint8_t  current_channel;
static uint32_t rx_last_count;
static int64_t  last_stat_ms, test_start_ms;
static bool     test_active;

static uint32_t swap_bits(uint32_t inp) {
	uint32_t r=0; inp&=0xFF;
	for(int i=0;i<8;i++) r|=((inp>>i)&1)<<(7-i);
	return r;
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

/* ---- 扫描定时器（仅未同步时使用）---- */
ISR_DIRECT_DECLARE(SCAN_TIMER_IRQ) {
	SCAN_TIMER->EVENTS_COMPARE[0]=0;
	if(!rx_synchronized) rx_channel_timeout=true;
	return 0;
}
static void scan_timer_start(uint32_t us) {
	SCAN_TIMER->EVENTS_COMPARE[0]=0;
	SCAN_TIMER->TASKS_CLEAR=1;
	SCAN_TIMER->CC[0]=us;
	SCAN_TIMER->TASKS_START=1;
}

/* ---- Radio ISR（极致精简）---- */
ISR_DIRECT_DECLARE(radio_isr) {
	if(NRF_RADIO->EVENTS_READY) { NRF_RADIO->EVENTS_READY=0; rx_ok=false; }
	if(NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END=0; rx_total++;
		if(NRF_RADIO->CRCSTATUS==1) {
			rx_ok=true;
			/* 读 RSSI 采样（值越大信号越强，单位 dBm 取反） */
			uint32_t rssi = NRF_RADIO->RSSISAMPLE;
			rx_rssi_sum += rssi; rx_rssi_cnt++;
			if(!rx_synchronized) {
				rx_synchronized=true; test_active=true;
				rx_seq_init=false;  /* ★ 新同步 → 重置序号基准 */
			}
			uint32_t seq=radio_packet[1]|((uint32_t)radio_packet[2]<<8)|((uint32_t)radio_packet[3]<<16)|((uint32_t)radio_packet[4]<<24);
			if(!rx_seq_init) {
				/* 首个序号，建立基准 */
				rx_seq_init=true; rx_last_seq=seq;
			} else if(seq == rx_last_seq) {
				rx_dup++;  /* 重复包（不应出现） */
			} else if(seq > rx_last_seq) {
				/* 序号间隙 = 丢失的包数 */
				uint32_t gap = seq - rx_last_seq - 1;
				rx_lost += gap;
				rx_last_seq = seq;
			} else {
				/* seq < rx_last_seq: 32-bit 回绕或乱序 */
				rx_lost += (0xFFFFFFFFu - rx_last_seq) + seq;
				rx_last_seq = seq;
			}
	}
	}
	if(NRF_RADIO->EVENTS_DISABLED) {
		NRF_RADIO->EVENTS_DISABLED=0;
		if(rx_ok) radio_hop();
	}
	if(NRF_RADIO->EVENTS_RXREADY) { NRF_RADIO->EVENTS_RXREADY=0; }
	return 0;
}

/* ---- Radio 初始化 ---- */
static void radio_init(void) {
	DP("R1m "); nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_4MBIT_BT_0_4);
	DP("R2f "); current_channel=0; nrf_radio_frequency_set(NRF_RADIO,2400+hop_channels[0]);
	DP("R3p "); nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);
	DP("R4a ");
	NRF_RADIO->PREFIX0=((uint32_t)swap_bits(0xC3)<<24)|((uint32_t)swap_bits(0xC2)<<16)|((uint32_t)swap_bits(0xC1)<<8)|((uint32_t)swap_bits(0xC0)<<0);
	NRF_RADIO->PREFIX1=((uint32_t)swap_bits(0xC7)<<24)|((uint32_t)swap_bits(0xC6)<<16)|((uint32_t)swap_bits(0xC4)<<0);
	NRF_RADIO->BASE0=bytewise_bitswap(0x01234567UL); NRF_RADIO->BASE1=bytewise_bitswap(0x89ABCDEFUL);
	NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1;
	DP("R5p ");
	NRF_RADIO->PCNF0=(0<<RADIO_PCNF0_S1LEN_Pos)|(0<<RADIO_PCNF0_S0LEN_Pos)|(0<<RADIO_PCNF0_LFLEN_Pos);
	NRF_RADIO->PCNF1=(RADIO_PCNF1_WHITEEN_Disabled<<RADIO_PCNF1_WHITEEN_Pos)|(RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos)|(4<<RADIO_PCNF1_BALEN_Pos)|(RADIO_PKT_LEN<<RADIO_PCNF1_STATLEN_Pos)|(RADIO_PKT_LEN<<RADIO_PCNF1_MAXLEN_Pos);
	DP("R6c "); NRF_RADIO->CRCCNF=(RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos); NRF_RADIO->CRCINIT=0xFFFFUL; NRF_RADIO->CRCPOLY=0x11021UL;
	DP("R7i ");
	NRF_RADIO->INTENSET00=RADIO_INTENSET00_READY_Msk|RADIO_INTENSET00_END_Msk|RADIO_INTENSET00_DISABLED_Msk|RADIO_INTENSET00_RXREADY_Msk;
	DP("R8q "); IRQ_DIRECT_CONNECT(RADIO_0_IRQn,0,radio_isr,0); NVIC_ClearPendingIRQ(RADIO_0_IRQn); irq_enable(RADIO_0_IRQn);
	DP("R9p "); NRF_RADIO->PACKETPTR=(uint32_t)radio_packet;
	DP("Ra "); nrf_radio_fast_ramp_up_enable_set(NRF_RADIO,true);
	DP("Rs "); NRF_RADIO->SHORTS=RADIO_SHORTS_RXREADY_START_Msk|RADIO_SHORTS_PHYEND_DISABLE_Msk|RADIO_SHORTS_DISABLED_RXEN_Msk|RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
	DP("done\n");
}

static void scan_timer_init(void) {
	DP("T1 "); SCAN_TIMER->TASKS_CLEAR=1; SCAN_TIMER->MODE=0; SCAN_TIMER->BITMODE=3; SCAN_TIMER->PRESCALER=5;
	DP("T2 "); SCAN_TIMER->CC[0]=1000; SCAN_TIMER->INTENSET=TIMER_INTENSET_COMPARE0_Msk; SCAN_TIMER->SHORTS=TIMER_SHORTS_COMPARE0_STOP_Msk;
	DP("T3 "); IRQ_DIRECT_CONNECT(SCAN_TIMER_IRQn,0,SCAN_TIMER_IRQ,0); NVIC_ClearPendingIRQ(SCAN_TIMER_IRQn); irq_enable(SCAN_TIMER_IRQn);
	DP("T4\n");
}

int main(void) {
	printk("\n=== nRF54LM20 Radio RX ===\n");
	DP("CLK "); const struct device *clk=DEVICE_DT_GET_OR_NULL(DT_NODELABEL(clock));
	if(!clk||!device_is_ready(clk)){printk("CLK FAIL\n");return 0;}
	int e=clock_control_on(clk,(clock_control_subsys_t)CLOCK_CONTROL_NRF_SUBSYS_HF);
	if(e<0){printk("CLK err %d\n",e);return 0;}
	DP("ok\n");

	radio_init(); scan_timer_init();
	dk_leds_init(); dk_set_led(DK_LED1,1);
	printk("Init done. Scanning...\n");
	scan_timer_start(1000);
	NRF_RADIO->TASKS_RXEN=1;

	while(1) {
		/* 未同步 → 扫描跳频 */
		if(!rx_synchronized && rx_channel_timeout) {
			rx_channel_timeout=false; rx_ok=false;
			NRF_RADIO->EVENTS_DISABLED=0; NRF_RADIO->TASKS_DISABLE=1;
			while(!NRF_RADIO->EVENTS_DISABLED){}
			radio_hop(); scan_timer_start(1000); NRF_RADIO->TASKS_RXEN=1;
		}

		/* 丢包检测（基于 cycle counter，不用 timer 寄存器）*/
		static uint8_t miss_cnt;
		if(rx_synchronized && !rx_ok) {
			miss_cnt++;
			if(miss_cnt>10) {
				rx_synchronized=false; miss_cnt=0;
				if(test_active) {
					int64_t el=k_uptime_get()-test_start_ms;
					uint32_t total=rx_total, lost=rx_lost, dup=rx_dup;
					uint32_t total_with_loss=total+lost;
					uint32_t pps=el>0?(uint32_t)(total*1000ULL/(uint64_t)el):0;
					printk("<<< Signal lost =====\n");
					printk("  ok:%u lost:%u dup:%u dur:%lldms rate:%upps\n",
						total, lost, dup, el, pps);
					if(total_with_loss>0) {
						uint32_t permille=lost*1000u/total_with_loss;
						printk("  loss rate: %u.%u%%\n", permille/10, permille%10);
					}
					test_active=false; rx_total=0; rx_lost=0; rx_dup=0;
				rx_last_count=0; rx_seq_init=false; rx_last_seq=0;
					dk_set_led(DK_LED1,1);
				}
			}
		} else if(rx_ok) { miss_cnt=0; }

		/* 刚同步 */
		if(rx_synchronized && !test_active) {
			test_active=true; rx_total=0; rx_lost=0; rx_dup=0; rx_last_count=0;
			rx_seq_init=false; rx_rssi_sum=0; rx_rssi_cnt=0;
			test_start_ms=last_stat_ms=k_uptime_get();
			printk(">>> Synced!\n"); dk_set_led(DK_LED3,1);
		}

		/* 每秒统计 */
		if(test_active) {
			int64_t now=k_uptime_get();
			if((now-last_stat_ms)>=1000) {
				uint32_t total=rx_total, lost=rx_lost, dup=rx_dup;
				uint32_t pps=total-rx_last_count;
				uint32_t total_with_loss = total + lost;
				/* 算平均 RSSI */
				int32_t avg_rssi=0;
				uint32_t cnt=rx_rssi_cnt;
				if(cnt>0) avg_rssi=-(int32_t)(rx_rssi_sum/cnt);
				rx_rssi_sum=0; rx_rssi_cnt=0;
				printk("[RX] +%u pkt/s | ok:%u lost:%u dup:%u rssi:%ddBm",
					pps, total, lost, dup, avg_rssi);
				if(total_with_loss>0) {
					uint32_t permille = lost*1000u/total_with_loss;
					printk(" (loss %u.%u%%)", permille/10, permille%10);
				}
				printk("\n");
				rx_last_count=total; last_stat_ms=now;
			}
		}
		k_sleep(K_MSEC(50));
	}
}
