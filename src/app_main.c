/*
  KaRadio 32
  A WiFi webradio player

Copyright (C) 2017  KaraWin

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <nvs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "driver/i2s.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/apps/netbiosns.h"
#include "mdns.h"

#include "app_main.h"
#include "common_tasks.h"
#include "common_int.h"

#include "spiram_fifo.h"
#include "audio_renderer.h"
#include "bt_config.h"
#include "audio_player.h"

#include <u8g2.h>
#include "u8g2_esp32_hal.h"
#include "addon.h"
#include "addonu8g2.h"

#include "ClickButtons.h"
#include "eeprom.h"
#include "bt_config.h"
#include "nvs_flash.h"
#include "gpio.h"
#include "wifi.h"
#include "servers.h"
#include "webclient.h"
#include "webserver.h"
#include "interface.h"
#include "vs1053.h"
#include "tas5805m.h"
#include "ClickEncoder.h"

#define TAG "main"

// Priorities of the reader and the decoder thread. bigger number = higher prio
#define PRIO_READER configMAX_PRIORITIES - 3
#define PRIO_MQTT configMAX_PRIORITIES - 3
#define PRIO_CONNECT configMAX_PRIORITIES - 1
#define striWATERMARK "watermark: %d  heap: %d"

/* */
// static bool wifiInitDone = false;
// static EventGroupHandle_t wifi_event_group;
xQueueHandle event_queue;

// xSemaphoreHandle print_mux;
static uint16_t FlashOn = 5, FlashOff = 5;
bool ledStatus;	  // true: normal blink, false: led on when playing
bool ledPolarity; // true: normal false: reverse
bool logTel;	  // true = log also on telnet
player_t *player_config;
static output_mode_t audio_output_mode;
static uint8_t clientIvol = 0;
// ip
static char localIp[20];
// 4MB sram?
static bool bigRam = false;
// timeout to save volume in flash
// static uint32_t ctimeVol = 0;
static uint32_t ctimeMs = 0;
static bool divide = false;

IRAM_ATTR char *getIp() { return (localIp); }
IRAM_ATTR uint8_t getIvol() { return clientIvol; }
IRAM_ATTR void setIvol(uint8_t vol) { clientIvol = vol; }; // ctimeVol = 0;}
IRAM_ATTR output_mode_t get_audio_output_mode() { return audio_output_mode; }

/*
IRAM_ATTR void   microsCallback(void *pArg) {
	int timer_idx = (int) pArg;
	queue_event_t evt;
	TIMERG1.hw_timer[timer_idx].update = 1;
	TIMERG1.int_clr_timers.t1 = 1; //isr ack
		evt.type = TIMER_1mS;
		evt.i1 = TIMERGROUP1mS;
		evt.i2 = timer_idx;
	xQueueSendFromISR(event_queue, &evt, NULL);
	TIMERG1.hw_timer[timer_idx].config.alarm_en = 1;
}*/

IRAM_ATTR bool bigSram()
{
	return bigRam;
}

//-----------------------------------
// every 500µs
IRAM_ATTR void msCallback(void *pArg)
{
	int timer_idx = (int)pArg;

	//	queue_event_t evt;
	TIMERG1.hw_timer[timer_idx].update = 1;
	TIMERG1.int_clr_timers.t0 = 1; // isr ack
	if (divide)
	{
		ctimeMs++; // for led
				   //		ctimeVol++; // to save volume
	}
	divide = !divide;
	if (serviceAddon != NULL)
		serviceAddon(); // for the encoders and buttons
	TIMERG1.hw_timer[timer_idx].config.alarm_en = 1;
}

IRAM_ATTR void sleepCallback(void *pArg)
{
	int timer_idx = (int)pArg;
	queue_event_t evt;
	TIMERG0.int_clr_timers.t0 = 1; // isr ack
	evt.type = TIMER_SLEEP;
	evt.i1 = TIMERGROUP;
	evt.i2 = timer_idx;
	xQueueSendFromISR(event_queue, &evt, NULL);
	TIMERG0.hw_timer[timer_idx].config.alarm_en = 0;
}

IRAM_ATTR void wakeCallback(void *pArg)
{

	int timer_idx = (int)pArg;
	queue_event_t evt;
	TIMERG0.int_clr_timers.t1 = 1;
	evt.i1 = TIMERGROUP;
	evt.i2 = timer_idx;
	evt.type = TIMER_WAKE;
	xQueueSendFromISR(event_queue, &evt, NULL);
	TIMERG0.hw_timer[timer_idx].config.alarm_en = 0;
}

// return the current timer value in sec
uint64_t getSleep()
{
	uint64_t ret = 0;
	uint64_t tot = 0;
	timer_get_alarm_value(TIMERGROUP, sleepTimer, &tot);
	timer_get_counter_value(TIMERGROUP, sleepTimer, &ret);
	ESP_LOGD(TAG, "getSleep: ret: %lld, tot: %lld, return %lld", ret, tot, tot - ret);
	return ((tot)-ret) / 5000000;
}

uint64_t getWake()
{
	uint64_t ret = 0;
	uint64_t tot = 0;
	timer_get_alarm_value(TIMERGROUP, wakeTimer, &tot);
	timer_get_counter_value(TIMERGROUP, wakeTimer, &ret);
	ESP_LOGD(TAG, "getWake: ret: %lld, tot: %lld  return %lld", ret, (tot), tot - ret);
	return ((tot)-ret) / 5000000;
}

void tsocket(const char *lab, uint32_t cnt)
{
	char *title = malloc(strlen(lab) + 50);
	sprintf(title, "{\"%s\":\"%d\"}", lab, cnt * 60);
	websocketbroadcast(title, strlen(title));
	free(title);
}

void stopSleep()
{
	ESP_LOGD(TAG, "stopSleep");
	ESP_ERROR_CHECK(timer_pause(TIMERGROUP, sleepTimer));
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMERGROUP, sleepTimer, 0x00000000ULL));
	ESP_ERROR_CHECK(timer_set_counter_value(TIMERGROUP, sleepTimer, 0x00000000ULL));

	tsocket("lsleep", 0);
}

void startSleep(uint32_t delay)
{
	ESP_LOGD(TAG, "startSleep: %d min.", delay);
	if (delay == 0)
		return;
	stopSleep();
	ESP_ERROR_CHECK(timer_set_counter_value(TIMERGROUP, sleepTimer, 0x00000000ULL));
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMERGROUP, sleepTimer, TIMERVALUE(delay * 60)));
	ESP_ERROR_CHECK(timer_enable_intr(TIMERGROUP, sleepTimer));
	ESP_ERROR_CHECK(timer_set_alarm(TIMERGROUP, sleepTimer, TIMER_ALARM_EN));
	ESP_ERROR_CHECK(timer_start(TIMERGROUP, sleepTimer));
	tsocket("lsleep", delay);
}

void stopWake()
{
	ESP_LOGD(TAG, "stopWake");
	ESP_ERROR_CHECK(timer_pause(TIMERGROUP, wakeTimer));
	ESP_ERROR_CHECK(timer_set_counter_value(TIMERGROUP, wakeTimer, 0x00000000ULL));
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMERGROUP, wakeTimer, 0x00000000ULL));
	tsocket("lwake", 0);
}

void startWake(uint32_t delay)
{
	ESP_LOGD(TAG, "startWake: %d min.", delay);
	if (delay == 0)
		return;
	stopWake();
	ESP_ERROR_CHECK(timer_set_counter_value(TIMERGROUP, wakeTimer, 0x00000000ULL));
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMERGROUP, wakeTimer, TIMERVALUE(delay * 60)));
	ESP_ERROR_CHECK(timer_enable_intr(TIMERGROUP, wakeTimer));
	ESP_ERROR_CHECK(timer_set_alarm(TIMERGROUP, wakeTimer, TIMER_ALARM_EN));
	ESP_ERROR_CHECK(timer_start(TIMERGROUP, wakeTimer));
	tsocket("lwake", delay);
}

void initTimers()
{
	timer_config_t config;
	config.alarm_en = 1;
	config.auto_reload = TIMER_AUTORELOAD_DIS;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_LEVEL;
	config.counter_en = TIMER_PAUSE;

	/*Configure timer sleep*/
	ESP_ERROR_CHECK(timer_init(TIMERGROUP, sleepTimer, &config));
	ESP_ERROR_CHECK(timer_pause(TIMERGROUP, sleepTimer));
	ESP_ERROR_CHECK(timer_isr_register(TIMERGROUP, sleepTimer, sleepCallback, (void *)sleepTimer, 0, NULL));
	/*Configure timer wake*/
	ESP_ERROR_CHECK(timer_init(TIMERGROUP, wakeTimer, &config));
	ESP_ERROR_CHECK(timer_pause(TIMERGROUP, wakeTimer));
	ESP_ERROR_CHECK(timer_isr_register(TIMERGROUP, wakeTimer, wakeCallback, (void *)wakeTimer, 0, NULL));
	/*Configure timer 1MS*/
	config.auto_reload = TIMER_AUTORELOAD_EN;
	config.divider = TIMER_DIVIDER1MS;
	ESP_ERROR_CHECK(timer_init(TIMERGROUP1MS, msTimer, &config));
	ESP_ERROR_CHECK(timer_pause(TIMERGROUP1MS, msTimer));
	ESP_ERROR_CHECK(timer_isr_register(TIMERGROUP1MS, msTimer, msCallback, (void *)msTimer, 0, NULL));
	/* start 1MS timer*/
	ESP_ERROR_CHECK(timer_set_counter_value(TIMERGROUP1MS, msTimer, 0x00000000ULL));
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMERGROUP1MS, msTimer, TIMERVALUE1MS(1)));
	ESP_ERROR_CHECK(timer_enable_intr(TIMERGROUP1MS, msTimer));
	ESP_ERROR_CHECK(timer_set_alarm(TIMERGROUP1MS, msTimer, TIMER_ALARM_EN));
	ESP_ERROR_CHECK(timer_start(TIMERGROUP1MS, msTimer));

	/*Configure timer 1µS*/
	/*	config.auto_reload = TIMER_AUTORELOAD_EN;
	config.divider = TIMER_DIVIDER1mS;
	ESP_ERROR_CHECK(timer_init(TIMERGROUP1mS, microsTimer, &config));
	ESP_ERROR_CHECK(timer_pause(TIMERGROUP1mS, microsTimer));
	ESP_ERROR_CHECK(timer_isr_register(TIMERGROUP1mS, microsTimer, microsCallback, (void*) microsTimer, 0, NULL));
*/
	/* start 1µS timer*/
	/*	ESP_ERROR_CHECK(timer_set_counter_value(TIMERGROUP1mS, microsTimer, 0x00000000ULL));
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMERGROUP1mS, microsTimer,TIMERVALUE1mS(10))); // 10 ms timer
	ESP_ERROR_CHECK(timer_enable_intr(TIMERGROUP1mS, microsTimer));
	ESP_ERROR_CHECK(timer_set_alarm(TIMERGROUP1mS, microsTimer,TIMER_ALARM_EN));
	ESP_ERROR_CHECK(timer_start(TIMERGROUP1mS, microsTimer));*/
}

//////////////////////////////////////////////////////////////////

// Renderer config creation
static renderer_config_t *create_renderer_config()
{
	renderer_config_t *renderer_config = calloc(1, sizeof(renderer_config_t));

	renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_16BIT;
	renderer_config->i2s_num = I2S_NUM_0;
	renderer_config->sample_rate = 44100;
	renderer_config->sample_rate_modifier = 1.0;
	renderer_config->output_mode = audio_output_mode;

	if (renderer_config->output_mode == I2S_MERUS)
	{
		renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_32BIT;
	}

	if (renderer_config->output_mode == DAC_BUILT_IN)
	{
		renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_16BIT;
	}

	return renderer_config;
}

/******************************************************************************
 * FunctionName : checkUart
 * Description  : Check for a valid uart baudrate
 * Parameters   : baud
 * Returns      : baud
 *******************************************************************************/
uint32_t checkUart(uint32_t speed)
{
	uint32_t valid[] = {1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 76880, 115200, 230400};
	int i;
	for (i = 0; i < 12; i++)
	{
		if (speed == valid[i])
			return speed;
	}
	return 115200; // default
}

static void VS1053_init_hardware()
{
	if (VS1053_HW_init()) // init spi
		VS1053_Start();

	ESP_LOGI(TAG, "VS1053 hardware initialized");
}

// blinking led and timer isr
void timerTask(void *p)
{
	//	struct device_settings *device;
	uint32_t cCur;
	bool stateLed = false;
	gpio_num_t gpioLed;
	//	int uxHighWaterMark;

	initTimers();
	bool isEsplay = option_get_esplay();
	gpio_get_ledgpio(&gpioLed);
	setLedGpio(gpioLed);

	if (gpioLed != GPIO_NONE)
	{
		gpio_output_conf(gpioLed);
		gpio_set_level(gpioLed, ledPolarity ? 1 : 0);
	}
	cCur = FlashOff * 10;

	queue_event_t evt;

	while (1)
	{
		// read and treat the timer queue events
		//		int nb = uxQueueMessagesWaiting(event_queue);
		//		if (nb >29) printf(" %d\n",nb);
		while (xQueueReceive(event_queue, &evt, 0))
		{
			switch (evt.type)
			{
			case TIMER_SLEEP:
				clientDisconnect("Timer"); // stop the player
				break;
			case TIMER_WAKE:
				clientConnect(); // start the player
				break;
			default:
				break;
			}
		}
		if (ledStatus)
		{
			if (ctimeMs >= cCur)
			{
				gpioLed = getLedGpio();

				if (stateLed)
				{
					if (gpioLed != GPIO_NONE)
						gpio_set_level(gpioLed, ledPolarity ? 1 : 0);
					stateLed = false;
					cCur = FlashOff * 10;
				}
				else
				{
					if (gpioLed != GPIO_NONE)
						gpio_set_level(gpioLed, ledPolarity ? 0 : 1);
					stateLed = true;
					cCur = FlashOn * 10;
				}
				ctimeMs = 0;
			}
		}

		vTaskDelay(10);

		if (isEsplay)
			rexp = i2c_keypad_read(); // read the expansion
	}

	vTaskDelete(NULL); // stop the task (never reached)
}

void uartInterfaceTask(void *pvParameters)
{
	char tmp[255];
	int d;
	uint8_t c;
	int t;
	//	struct device_settings *device;
	uint32_t uspeed;
	int uxHighWaterMark;

	uspeed = g_device->uartspeed;
	uart_config_t uart_config0 = {
		.baud_rate = uspeed,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // UART_HW_FLOWCTRL_CTS_RTS,
		.rx_flow_ctrl_thresh = 0,
	};
	uart_param_config(UART_NUM_0, &uart_config0);
	uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

	for (t = 0; t < sizeof(tmp); t++)
		tmp[t] = 0;
	t = 0;

	while (1)
	{
		while (1)
		{
			d = uart_read_bytes(UART_NUM_0, &c, 1, 100);
			if (d > 0)
			{
				if ((char)c == '\r')
					break;
				if ((char)c == '\n')
					break;
				tmp[t] = (char)c;
				t++;
				if (t == sizeof(tmp) - 1)
					t = 0;
			}
		}
		checkCommand(t, tmp);
		uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
		ESP_LOGD("uartInterfaceTask", striWATERMARK, uxHighWaterMark, xPortGetFreeHeapSize());

		for (t = 0; t < sizeof(tmp); t++)
			tmp[t] = 0;
		t = 0;
	}
}

// In STA mode start a station or start in pause mode.
// Show ip on AP mode.
void autoPlay()
{
	char apmode[50];
	ESP_LOGI(TAG, "%s at IP %s", apmode, localIp);
	if (g_device->current_ap == APMODE)
	{
		clientSaveOneHeader("Configure the AP with the web page", 34, METANAME);
		clientSaveOneHeader(apmode, strlen(apmode), METAGENRE);
	}
	else
	{
		clientSaveOneHeader(apmode, strlen(apmode), METANAME);
		if ((audio_output_mode == VS1053) && (getVsVersion() < 3))
		{
			clientSaveOneHeader("Invalid audio output. VS1053 not found", 38, METAGENRE);
			ESP_LOGE(TAG, "Invalid audio output. VS1053 not found");
			vTaskDelay(200);
		}

		setCurrentStation(g_device->currentstation);
		if ((g_device->autostart == 1) && (g_device->currentstation != 0xFFFF))
		{
			kprintf("autostart: playing:%d, currentstation:%d\n", g_device->autostart, g_device->currentstation);
			vTaskDelay(10); // wait a bit
			playStationInt(g_device->currentstation);
		}
		else
			clientSaveOneHeader("Ready", 5, METANAME);
	}
}

void config_mdns(void)
{
	// initialize mDNS service
	esp_err_t err = mdns_init();
	if (err)
		ESP_LOGE(TAG, "mDNS Init failed: %d", err);
	else
		ESP_LOGI(TAG, "mDNS Init ok");

	// set hostname and instance name
	if ((strlen(g_device->hostname) == 0) || (strlen(g_device->hostname) > HOSTLEN))
	{
		strcpy(g_device->hostname, CONFIG_HOSTNAME);
	}

	ESP_LOGI(TAG, "mDNS Hostname: %s", g_device->hostname);
	err = mdns_hostname_set(g_device->hostname);
	if (err)
		ESP_LOGE(TAG, "Hostname Init failed: %d", err);

	ESP_ERROR_CHECK(mdns_instance_name_set(g_device->hostname));
	ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
	ESP_ERROR_CHECK(mdns_service_add(NULL, "_telnet", "_tcp", 23, NULL, 0));

	netbiosns_init();
    netbiosns_set_name(g_device->hostname);
}

/**
 * Main entry point
 */
void app_main()
{
	uint32_t uspeed;
	xTaskHandle pxCreatedTask;
	esp_err_t err;

	ESP_LOGI(TAG, "starting app_main()");
	ESP_LOGI(TAG, "RAM left: %u", esp_get_free_heap_size());

	const esp_partition_t *running = esp_ota_get_running_partition();
	ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
			 running->type, running->subtype, running->address);

	// Initialize NVS.
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		// OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialization to fail.
		// If this happens, we erase NVS partition and initialize NVS again.
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	// Check if we are in large Sram config
	if (xPortGetFreeHeapSize() > 0x80000)
		bigRam = true;

	// init hardware
	partitions_init();

	ESP_LOGI(TAG, "Partition init done...");

	if (g_device->cleared != 0xAABB)
	{
		ESP_LOGE(TAG, "Device config not ok. Try to restore");
		free(g_device);
		restoreDeviceSettings(); // try to restore the config from the saved one
		g_device = getDeviceSettings();
		if (g_device->cleared != 0xAABB)
		{
			ESP_LOGE(TAG, "Device config not cleared. Clear it.");
			free(g_device);
			eeEraseAll();
			g_device = getDeviceSettings();
			g_device->cleared = 0xAABB;	  // marker init done
			g_device->uartspeed = 115200; // default
			option_get_audio_output(&(g_device->audio_output_mode));
			g_device->trace_level = ESP_LOG_DEBUG; //default
			g_device->vol = 100;				   //default
			g_device->led_gpio = GPIO_NONE;
			saveDeviceSettings(g_device);
		}
		else
			ESP_LOGE(TAG, "Device config restored");
	}

	copyDeviceSettings(); // copy in the safe partion

	// Configure Deep Sleep start and wakeup options
	deepSleepConf(); // also called in addon.c

	// Enter ESP32 Deep Sleep (but not powerdown uninitialized peripherals) when P_SLEEP GPIO is P_LEVEL_SLEEP
	if (checkDeepSleepInput())
		esp_deep_sleep_start();

	// led mode
	if (g_device->options & T_LED)
		ledStatus = false; // play mode
	else
		ledStatus = true; // blink mode

	if (g_device->options & T_LEDPOL)
		ledPolarity = true;
	else
		ledPolarity = false;

	// log on telnet
	if (g_device->options & T_LOGTEL)
		logTel = true; //
	else
		logTel = false; //

	// init softwares
	telnetinit();
	websocketinit();

	// log level
	setLogLevel(g_device->trace_level);

	// time display
	uint8_t ddmm;
	option_get_ddmm(&ddmm);
	setDdmm(ddmm ? 1 : 0);

	// SPI init for the vs1053 and lcd if spi.
	VS1053_spi_init();
	VS1053_init_hardware();

	// the esplay board needs I2C for gpio extension
	if (option_get_esplay())
	{
		gpio_num_t scl;
		gpio_num_t sda;
		gpio_num_t rsti2c;
		gpio_get_i2c(&scl, &sda, &rsti2c);
		ESP_LOGD(TAG, "I2C GPIO SDA: %d, SCL: %d", sda, scl);
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_io_num = sda;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_io_num = scl;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
		esp_err_t res = i2c_param_config(I2C_MASTER_NUM, &conf);
		ESP_LOGD(TAG, "I2C setup : %d\n", res);
		res = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
		if (res != 0)
			ESP_LOGD(TAG, "I2C already installed. No problem");
		else
			ESP_LOGD(TAG, "I2C installed: %d", res);

		// init the amp shutdown gpio4 as output level 1
		gpio_output_conf(PIN_AUDIO_SHDN);
	}
	else
	{
		ESP_LOGI(TAG, "Skipping esplay configuration");
	}

	if (option_get_tas5805m())
	{
		gpio_num_t scl;
		gpio_num_t sda;
		gpio_num_t rsti2c;
		gpio_get_i2c(&scl, &sda, &rsti2c);
		ESP_LOGD(TAG, "I2C GPIO SDA: %d, SCL: %d", sda, scl);
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_io_num = sda;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_io_num = scl;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
		// ESP_ERROR_CHECK
		(i2c_param_config(I2C_MASTER_NUM, &conf));
		ESP_LOGD(TAG, "i2c_driver_install %d", I2C_MASTER_NUM);
		// ESP_ERROR_CHECK
		(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

		/* init amp */
		ESP_ERROR_CHECK(tas5806m_init());
		/* task for the audio amp */
		xTaskCreatePinnedToCore(&tas5806m_task, "audioamp_task", 8192, NULL, PRIO_AMP, &pxCreatedTask, CPU_AMP);
		ESP_LOGI(TAG, "%s task: %x", "audioamp_task", (unsigned int)pxCreatedTask);
	}
	else
	{
		ESP_LOGI(TAG, "Skipping tas5805m configuration");
	}

	/*
	// Init i2c if lcd doesn't not (spi) for rde5807=
	if (g_device->lcd_type >= LCD_SPI)
	{
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_io_num = (g_device->lcd_type == LCD_NONE)?PIN_I2C_SDA:PIN_SI2C_SDA;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_io_num = (g_device->lcd_type == LCD_NONE)?PIN_I2C_SCL:PIN_SI2C_SCL;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = I2C_MASTER_RFREQ_HZ;
		//ESP_ERROR_CHECK
		(i2c_param_config(I2C_MASTER_NUM, &conf));
		ESP_LOGD(TAG, "i2c_driver_install %d", I2C_MASTER_NUM);
		//ESP_ERROR_CHECK
		(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
	}
*/

	// output mode
	// I2S, I2S_MERUS, DAC_BUILT_IN, PDM, VS1053
	audio_output_mode = g_device->audio_output_mode;
	ESP_LOGI(TAG, "audio_output_mode %d\nOne of I2S=0, I2S_MERUS, DAC_BUILT_IN, PDM, VS1053, SPDIF", audio_output_mode);

	//Initialize the SPI RAM chip communications and see if it actually retains some bytes. If it doesn't, warn user.
	ramInit();

	// uart speed
	uspeed = g_device->uartspeed;
	uspeed = checkUart(uspeed);
	uart_set_baudrate(UART_NUM_0, uspeed);
	ESP_LOGI(TAG, "Set baudrate at %d", uspeed);
	if (g_device->uartspeed != uspeed)
	{
		g_device->uartspeed = uspeed;
		saveDeviceSettings(g_device);
	}

	// Version infos
	ESP_LOGI(TAG, "Release %s, Revision %s", RELEASE, REVISION);
	ESP_LOGI(TAG, "SDK %s", esp_get_idf_version());
	ESP_LOGI(TAG, "Heap size: %d", xPortGetFreeHeapSize());

	// lcd init
	uint8_t rt;
	option_get_lcd_info(&g_device->lcd_type, &rt);
	ESP_LOGI(TAG, "LCD Type %d", g_device->lcd_type);
	// lcd rotation
	setRotat(rt);
	lcd_init(g_device->lcd_type);
	ESP_LOGI(TAG, "Hardware init done...");

	lcd_welcome("", "");
	lcd_welcome("", "STARTING");

	// volume
	setIvol(g_device->vol);
	ESP_LOGI(TAG, "Volume set to %d", g_device->vol);

	// queue for events of the sleep / wake and Ms timers
	event_queue = xQueueCreate(30, sizeof(queue_event_t));
	// led blinks
	xTaskCreatePinnedToCore(timerTask, "timerTask", 2100, NULL, PRIO_TIMER, &pxCreatedTask, CPU_TIMER);
	ESP_LOGD(TAG, "%s task: %x", "t0", (unsigned int)pxCreatedTask);

	//-----------------------------
	// start the network
	//-----------------------------
	/* init wifi & network*/
	// start_wifi();
	// start_network();
	wifi_init_sta();

	//-----------------------------------------------------
	// init softwares
	//-----------------------------------------------------
	clientInit();

	config_mdns();

	// init player config
	player_config = (player_t *)calloc(1, sizeof(player_t));
	player_config->command = CMD_NONE;
	player_config->decoder_status = UNINITIALIZED;
	player_config->decoder_command = CMD_NONE;
	player_config->buffer_pref = BUF_PREF_SAFE;
	player_config->media_stream = calloc(1, sizeof(media_stream_t));

	audio_player_init(player_config);
	renderer_init(create_renderer_config());

	// LCD Display infos
	lcd_welcome(localIp, "STARTED");
	vTaskDelay(10);
	ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());

	// start tasks of KaRadio32
	xTaskCreatePinnedToCore(uartInterfaceTask, "uartInterfaceTask", 2500, NULL, PRIO_UART, &pxCreatedTask, CPU_UART);
	ESP_LOGI(TAG, "%s task: %x", "uartInterfaceTask", (unsigned int)pxCreatedTask);
	vTaskDelay(1);

	xTaskCreatePinnedToCore(clientTask, "clientTask", 4096, NULL, PRIO_CLIENT, &pxCreatedTask, CPU_CLIENT);
	ESP_LOGI(TAG, "%s task: %x", "clientTask", (unsigned int)pxCreatedTask);
	vTaskDelay(1);

	xTaskCreatePinnedToCore(serversTask, "serversTask", 3100, NULL, PRIO_SERVER, &pxCreatedTask, CPU_SERVER);
	ESP_LOGI(TAG, "%s task: %x", "serversTask", (unsigned int)pxCreatedTask);
	vTaskDelay(1);

	xTaskCreatePinnedToCore(task_addon, "task_addon", 2200, NULL, PRIO_ADDON, &pxCreatedTask, CPU_ADDON);
	ESP_LOGI(TAG, "%s task: %x", "task_addon", (unsigned int)pxCreatedTask);

	vTaskDelay(60); // wait tasks init
	ESP_LOGI(TAG, " Init Done");

	setIvol(g_device->vol);
	kprintf("READY. Type help for a list of commands\n");
	// error log on telnet
	esp_log_set_vprintf((vprintf_like_t)lkprintf);

	// autostart
	autoPlay();
}