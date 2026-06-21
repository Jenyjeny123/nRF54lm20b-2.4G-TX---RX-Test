/*
 * nRF54LM20 2.4G 吞吐量测试 —— TX (裸 Radio + DPPI 8000Hz)
 * 基于 LRchangyu/nrf54l_radio 移植
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

LOG_MODULE_REGISTER(tx_radio, LOG_LEVEL_INF);

/* 诊断宏：打印后忙等 1ms 确保 UART FIFO 排空 */
#define DP(...) do { printk(__VA_ARGS__); k_busy_wait(1000); } while(0)

/* ===== 包类型 ===== */
#define TYPE_START  0x01
#define TYPE_STOP   0x02
#define TYPE_DATA   0x03

/* ===== Radio 配置 ===== */
#define RADIO_PKT_LEN    8
#define HOP_CHANNELS_NUM 8
static const uint8_t hop_channels[HOP_CHANNELS_NUM] = {2,14,36,56,66,78,0,22};

/* ===== DPPI 通道 ===== */
#define DPPI_CH_TXEN      2
#define DPPI_CH_DISABLE   3
#define DPPI20_CH_TX_IND     0
#define DPPI20_CH_TX_DISABLE 1
#define TX_IND_PIN      (32+9)
#define TX_DISABLE_PIN  (32+11)

/* ===== 全局状态 ===== */
static volatile bool test_running, btn0_pressed;
static int64_t btn_cooldown_ms = -2000;
static uint32_t tx_total, tx_last_count;
static int64_t  last_stat_ms, test_start_ms;
static uint8_t  radio_packet[RADIO_PKT_LEN];
static uint8_t  current_channel;

/* ---- 位翻转 ---- */
static uint32_t swap_bits(uint32_t inp) {
	uint32_t r=0; inp&=0xFF;
	for(int i=0;i<8;i++) r|=((inp>>i)&1)<<(7-i);
	return r;
}
static uint32_t bytewise_bitswap(uint32_t inp) {
	return (swap_bits(inp>>24)<<24)|(swap_bits(inp>>16)<<16)|(swap_bits(inp>>8)<<8)|swap_bits(inp);
}

/* ---- 跳频 ---- */
static void radio_hop(void) {
	current_channel = (current_channel+1) % HOP_CHANNELS_NUM;
	*(volatile uint32_t*)((uint8_t*)NRF_RADIO+0x70C) &= ~(1u<<31);
	nrf_radio_frequency_set(NRF_RADIO, 2400+hop_channels[current_channel]);
	*(volatile uint32_t*)((uint8_t*)NRF_RADIO+0x07C) = 1;
}

/* ---- Radio ISR ---- */
ISR_DIRECT_DECLARE(radio_isr) {
	if(NRF_RADIO->EVENTS_READY)  { NRF_RADIO->EVENTS_READY=0; }
	if(NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END=0;
		if(test_running) {
			tx_total++;
			radio_hop();
			radio_packet[0]=TYPE_DATA;
			radio_packet[1]=(uint8_t)(tx_total&0xFF);
			radio_packet[2]=(uint8_t)((tx_total>>8)&0xFF);
			radio_packet[3]=(uint8_t)((tx_total>>16)&0xFF);
			radio_packet[4]=(uint8_t)((tx_total>>24)&0xFF);
		}
	}
	if(NRF_RADIO->EVENTS_DISABLED){ NRF_RADIO->EVENTS_DISABLED=0; }
	if(NRF_RADIO->EVENTS_RXREADY) { NRF_RADIO->EVENTS_RXREADY=0; }
	return 0;
}

/* ---- Radio 初始化 ---- */
static void radio_init(void) {
	DP("R1 mode ");
	nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_4MBIT_BT_0_4);
	DP("R2 freq ");
	current_channel=0;
	nrf_radio_frequency_set(NRF_RADIO, 2400+hop_channels[0]);
	DP("R3 txpwr ");
	nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);
	DP("R4 addr ");
	NRF_RADIO->PREFIX0=((uint32_t)swap_bits(0xC3)<<24)|((uint32_t)swap_bits(0xC2)<<16)|((uint32_t)swap_bits(0xC1)<<8)|((uint32_t)swap_bits(0xC0)<<0);
	NRF_RADIO->PREFIX1=((uint32_t)swap_bits(0xC7)<<24)|((uint32_t)swap_bits(0xC6)<<16)|((uint32_t)swap_bits(0xC4)<<0);
	NRF_RADIO->BASE0=bytewise_bitswap(0x01234567UL);
	NRF_RADIO->BASE1=bytewise_bitswap(0x89ABCDEFUL);
	NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1;
	DP("R5 pcnf ");
	NRF_RADIO->PCNF0=(0<<RADIO_PCNF0_S1LEN_Pos)|(0<<RADIO_PCNF0_S0LEN_Pos)|(0<<RADIO_PCNF0_LFLEN_Pos);
	NRF_RADIO->PCNF1=(RADIO_PCNF1_WHITEEN_Disabled<<RADIO_PCNF1_WHITEEN_Pos)|(RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos)|(4<<RADIO_PCNF1_BALEN_Pos)|(RADIO_PKT_LEN<<RADIO_PCNF1_STATLEN_Pos)|(RADIO_PKT_LEN<<RADIO_PCNF1_MAXLEN_Pos);
	DP("R6 crc ");
	NRF_RADIO->CRCCNF=(RADIO_CRCCNF_LEN_Two<<RADIO_CRCCNF_LEN_Pos);
	NRF_RADIO->CRCINIT=0xFFFFUL; NRF_RADIO->CRCPOLY=0x11021UL;
	DP("R7 int ");
	NRF_RADIO->INTENSET00=RADIO_INTENSET00_READY_Msk|RADIO_INTENSET00_END_Msk|RADIO_INTENSET00_DISABLED_Msk|RADIO_INTENSET00_RXREADY_Msk;
	DP("R8 irq ");
	IRQ_DIRECT_CONNECT(RADIO_0_IRQn, 0, radio_isr, 0);
	NVIC_ClearPendingIRQ(RADIO_0_IRQn);
	irq_enable(RADIO_0_IRQn);
	DP("R9 ptr ");
	NRF_RADIO->PACKETPTR=(uint32_t)radio_packet;
	DP("R10 ramp ");
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, true);
	DP("R11 shorts ");
	NRF_RADIO->SHORTS=RADIO_SHORTS_TXREADY_START_Msk|RADIO_SHORTS_PHYEND_DISABLE_Msk;
	DP("R12 done\n");
}

/* ---- DPPI 初始化 ---- */
static void dppi_tx_chain_init(void) {
	DP("D1 timer ");
	NRF_TIMER10->TASKS_CLEAR=1;
	NRF_TIMER10->MODE=0; NRF_TIMER10->BITMODE=3; NRF_TIMER10->PRESCALER=5;
	NRF_TIMER10->CC[0]=125;
	NRF_TIMER10->SHORTS=TIMER_SHORTS_COMPARE0_CLEAR_Msk;
	DP("D2 pub ");
	NRF_TIMER10->PUBLISH_COMPARE[0]=((1u<<31)|DPPI_CH_TXEN);
	NRF_RADIO->SUBSCRIBE_TXEN=((1u<<31)|DPPI_CH_TXEN);
	NRF_RADIO->PUBLISH_DISABLED=((1u<<31)|DPPI_CH_DISABLE);
	DP("D3 ch10 ");
	NRF_DPPIC10->CHEN=(1<<DPPI_CH_TXEN)|(1<<DPPI_CH_DISABLE);
	DP("D4 pp11 ");
	NRF_PPIB11->SUBSCRIBE_SEND[0]=((1u<<31)|DPPI_CH_TXEN);
	NRF_PPIB11->SUBSCRIBE_SEND[1]=((1u<<31)|DPPI_CH_DISABLE);
	DP("D5 pp21 ");
	NRF_PPIB21->PUBLISH_RECEIVE[0]=((1u<<31)|DPPI20_CH_TX_IND);
	NRF_PPIB21->PUBLISH_RECEIVE[1]=((1u<<31)|DPPI20_CH_TX_DISABLE);
	DP("D6 ch20 ");
	NRF_DPPIC20->CHEN=(1<<DPPI20_CH_TX_IND)|(1<<DPPI20_CH_TX_DISABLE);
	DP("D7 gpio ");
	nrf_gpio_cfg_output(TX_IND_PIN); nrf_gpio_pin_clear(TX_IND_PIN);
	nrf_gpio_cfg_output(TX_DISABLE_PIN); nrf_gpio_pin_clear(TX_DISABLE_PIN);
	NRF_GPIOTE20->CONFIG[0]=(3|(TX_IND_PIN<<4)|(3<<16));
	NRF_GPIOTE20->SUBSCRIBE_OUT[0]=((1u<<31)|DPPI20_CH_TX_IND);
	NRF_GPIOTE20->CONFIG[1]=(3|(TX_DISABLE_PIN<<4)|(3<<16));
	NRF_GPIOTE20->SUBSCRIBE_OUT[1]=((1u<<31)|DPPI20_CH_TX_DISABLE);
	DP("D8 done\n");
}

/* ---- 按钮回调 ---- */
static void btn_handler(uint32_t pressed, uint32_t changed) {
	if(pressed & changed & DK_BTN1_MSK) btn0_pressed=true;
}

/* ---- 打印统计 ---- */
static void print_stats(void) {
	int64_t el=k_uptime_get()-test_start_ms;
	uint32_t pps=el>0?(uint32_t)(tx_total*1000ULL/(uint64_t)el):0;
	printk("===== TX STATS =====\n  sent:%u  dur:%lldms  rate:%upps\n", tx_total, el, pps);
}

/* ================================================================ */
int main(void) {
	printk("\n=== nRF54LM20 Radio TX 8K DPPI ===\n");

	/* HFCLK */
	DP("CLK start ");
	const struct device *clk=DEVICE_DT_GET_OR_NULL(DT_NODELABEL(clock));
	if(!clk||!device_is_ready(clk)) { printk("CLK FAIL\n"); return 0; }
	int e=clock_control_on(clk, (clock_control_subsys_t)CLOCK_CONTROL_NRF_SUBSYS_HF);
	if(e<0) { printk("CLK on err %d\n",e); return 0; }
	DP("CLK ok\n");

	/* Radio */
	radio_init();

	/* DPPI */
	dppi_tx_chain_init();

	/* Button + LED */
	dk_leds_init();
	dk_buttons_init(btn_handler);
	dk_set_led(DK_LED1,1);
	printk("Init complete. Press Button 0.\n");

	memset(radio_packet,0,RADIO_PKT_LEN);
	radio_packet[0]=TYPE_DATA;

	while(1) {
		if(btn0_pressed && (k_uptime_get()-btn_cooldown_ms>1000)) {
			btn0_pressed=false; btn_cooldown_ms=k_uptime_get();
			if(!test_running) {
				printk(">>> TEST START\n");
				tx_total=0; tx_last_count=0; test_start_ms=last_stat_ms=k_uptime_get();
				test_running=true;
				NRF_TIMER10->TASKS_START=1;
				dk_set_leds(DK_LED3_MSK);
			} else {
				NRF_TIMER10->TASKS_STOP=1;
				test_running=false;
				printk("<<< TEST STOP\n");
				print_stats();
				dk_set_leds(DK_LED1_MSK);
			}
		}
		if(test_running) {
			int64_t now=k_uptime_get();
			if((now-last_stat_ms)>=1000) {
				uint32_t pps=tx_total-tx_last_count;
				printk("[TX] +%u pkt/s | total:%u\n", pps, tx_total);
				tx_last_count=tx_total; last_stat_ms=now;
			}
		}
		k_sleep(K_MSEC(10));
	}
}
