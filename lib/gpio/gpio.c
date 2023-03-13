/******************************************************************************
 *
 * Copyright 2017 karaw(http://www.karawin.fr)
 *
 *******************************************************************************/

// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#define TAG "GPIO"
#include "driver/gpio.h"
#include "gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "eeprom.h"

// static xSemaphoreHandle muxnvs = NULL;
// const char hardware[] = {"hardware"};
// const char option_space[] = {"option_space"};
// const char gpio_space[] = {"gpio_space"};
// const char label_space[] = {"label_space"};

// init a gpio as output
void gpio_output_conf(gpio_num_t gpio)
{
	gpio_config_t gpio_conf;
	gpio_conf.pin_bit_mask = ((uint64_t)(((uint64_t)1) << gpio));
	gpio_conf.mode = GPIO_MODE_OUTPUT;
	gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_conf.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&gpio_conf);
	gpio_set_level(gpio, 1);
}

// open and read the gpio hardware setting
//
// esp_err_t open_partition(const char *partition_label, const char *namespace, nvs_open_mode open_mode, nvs_handle *handle)
// {
// 	esp_err_t err;
// 	if (muxnvs == NULL)
// 		muxnvs = xSemaphoreCreateMutex();
// 	xSemaphoreTake(muxnvs, portMAX_DELAY);
// 	err = nvs_flash_init_partition(partition_label);
// 	if (err != ESP_OK)
// 	{
// 		ESP_LOGE(TAG, "Hardware partition not found");
// 		return err;
// 	}

// 	//	ESP_ERROR_CHECK(nvs_open_from_partition(partition_label, namespace, open_mode, handle));
// 	err = nvs_open_from_partition(partition_label, namespace, open_mode, handle);
// 	if (err != ESP_OK)
// 	{
// 		ESP_LOGE(TAG, "Namespace %s not found, ERR: %x", namespace, err);
// 		nvs_flash_deinit_partition(partition_label);
// 		xSemaphoreGive(muxnvs);
// 	}
// 	return err;
// }

// void close_partition(nvs_handle handle, const char *partition_label)
// {
// 	nvs_commit(handle); // if a write is pending
// 	nvs_close(handle);
// 	nvs_flash_deinit_partition(partition_label);
// 	xSemaphoreGive(muxnvs);
// }

char* gpio_get_label()
{
	// size_t required_size;
	// nvs_handle hardware_handle;
	// *label = NULL;
	// if (open_partition(hardware, label_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "get label");
	// 	return;
	// }
	// nvs_get_str(hardware_handle, "L_LABEL", NULL, &required_size);
	// if (required_size > 1)
	// {
	// 	*label = malloc(required_size);
	// 	nvs_get_str(hardware_handle, "L_LABEL", *label, &required_size);
	// 	ESP_LOGV(TAG, "Label: \"%s\"\n Required size: %d", *label, required_size);
	// }
	// close_partition(hardware_handle, hardware);
	return CONFIG_L_LABEL;
}

char* gpio_get_comment()
{
	// size_t required_size;
	// nvs_handle hardware_handle;
	// *label = NULL;
	// if (open_partition(hardware, label_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "get comment");
	// 	return;
	// }
	// nvs_get_str(hardware_handle, "L_COMMENT", NULL, &required_size);
	// if (required_size > 1)
	// {
	// 	*label = malloc(required_size);
	// 	nvs_get_str(hardware_handle, "L_COMMENT", *label, &required_size);
	// 	ESP_LOGV(TAG, "Label: \"%s\"\n Required size: %d", *label, required_size);
	// }
	// close_partition(hardware_handle, hardware);
	return CONFIG_L_COMMENT;
}

void gpio_get_spi_bus(uint8_t *spi_no, gpio_num_t *miso, gpio_num_t *mosi, gpio_num_t *sclk)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// if (miso != NULL)
	// 	*miso = PIN_NUM_MISO;
	// if (mosi != NULL)
	// 	*mosi = PIN_NUM_MOSI;
	// if (sclk != NULL)
	// 	*sclk = PIN_NUM_CLK;
	// if (spi_no != NULL)
	// 	*spi_no = VSPI_HOST;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "spi_bus");
	// 	return;
	// }
	// err = ESP_OK;
	// if (spi_no != NULL)
	// 	err = nvs_get_u8(hardware_handle, "K_SPI", (uint8_t *)spi_no);
	// if (miso != NULL)
	// 	err |= nvs_get_u8(hardware_handle, "P_MISO", (uint8_t *)miso);
	// if (mosi != NULL)
	// 	err |= nvs_get_u8(hardware_handle, "P_MOSI", (uint8_t *)mosi);
	// if (sclk != NULL)
	// 	err |= nvs_get_u8(hardware_handle, "P_CLK", (uint8_t *)sclk);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_spi_bus err 0x%x", err);
	// close_partition(hardware_handle, hardware);
	if (miso)
	#ifdef CONFIG_P_MISO
	*miso = CONFIG_P_MISO;
	#else 
	*miso = PIN_NUM_MISO;
	#endif

	if (mosi)
	#ifdef CONFIG_P_MOSI
	*mosi = CONFIG_P_MOSI;
	#else 
	*mosi = PIN_NUM_MOSI;
	#endif

	if (sclk)
	#ifdef CONFIG_P_CLK
	*sclk = CONFIG_P_CLK;
	#else
	*sclk = PIN_NUM_CLK;
	#endif

	if (spi_no)
	#ifdef CONFIG_K_SPI
	*spi_no = CONFIG_K_SPI;
	#else
	*spi_no = KSPI;
	#endif
}

void gpio_get_vs1053(gpio_num_t *xcs, gpio_num_t *rst, gpio_num_t *xdcs, gpio_num_t *dreq)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *xcs = PIN_NUM_XCS;
	// *rst = PIN_NUM_RST;
	// *xdcs = PIN_NUM_XDCS;
	// *dreq = PIN_NUM_DREQ;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "vs1053");
	// 	return;
	// }
	// err = nvs_get_u8(hardware_handle, "P_XCS", (uint8_t *)xcs);
	// err |= nvs_get_u8(hardware_handle, "P_RST", (uint8_t *)rst);
	// err |= nvs_get_u8(hardware_handle, "P_XDCS", (uint8_t *)xdcs);
	// err |= nvs_get_u8(hardware_handle, "P_DREQ", (uint8_t *)dreq);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_vs1053 err 0x%x", err);
	// close_partition(hardware_handle, hardware);

	#ifdef CONFIG_P_XCS
	*xcs = CONFIG_P_XCS;
	#else
	*xcs = PIN_NUM_XCS;
	#endif

	#ifdef CONFIG_P_RST
	*rst = CONFIG_P_RST;
	#else
	*rst = PIN_NUM_RST;
	#endif

	#ifdef CONFIG_P_XDCS
	*xdcs = CONFIG_P_XDCS;
	#else
	*xdcs = PIN_NUM_XDCS;
	#endif

	#ifdef CONFIG_P_DREQ
	*dreq = CONFIG_P_DREQ;
	#else
	*dreq = PIN_NUM_DREQ;
	#endif
}

bool option_get_tas5805m()
{
	// esp_err_t err;
	// bool ret = false;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, option_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "get_tas5805m");
	// 	return ret;
	// }
	// err = nvs_get_u8(hardware_handle, "O_TAS5805M", (uint8_t *)&ret);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "get_tas5805m err 0x%x", err);
	// close_partition(hardware_handle, hardware);
	// return ret;
	#ifdef CONFIG_O_TAS5805M
	return true;
	#else
	return false;
	#endif
}

void option_get_audio_output(output_mode_t *oom)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *oom = I2S;
	// if (open_partition(hardware, option_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "get_audio");
	// 	return;
	// }
	// err = nvs_get_u8(hardware_handle, "O_AUDIO", (uint8_t *)oom);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "get_audio err 0x%x", err);
	// close_partition(hardware_handle, hardware);
	*oom = CONFIG_O_AUDIO;
}

bool option_get_esplay()
{
	// esp_err_t err;
	// bool ret = false;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, option_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "get_esplay");
	// 	return ret;
	// }
	// err = nvs_get_u8(hardware_handle, "O_ESPLAY", (uint8_t *)&ret);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "get_esplay err 0x%x", err);
	// close_partition(hardware_handle, hardware);
	// return ret;
	#ifdef CONFIG_O_ESPLAY
	return true;
	#else
	return false;
	#endif
}

void option_get_lcd_info(uint8_t *enca, uint8_t *rt)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// uint8_t typ, rot;
	// // init default
	// *enca = g_device->lcd_type;
	// *rt = ((g_device->options32) & T_ROTAT) ? 1 : 0;
	// if (open_partition(hardware, option_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "lcd_info");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "O_LCD_TYPE", (uint8_t *)&typ);
	// err |= nvs_get_u8(hardware_handle, "O_LCD_ROTA", (uint8_t *)&rot);
	// if (typ != 255)
	// 	*enca = typ;
	// if (rot != 255)
	// 	*rt = rot;
	// if (*rt)
	// 	*rt = 1;
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oget_lcd_info err 0x%x", err);
	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_O_LCD_TYPE
	*enca = CONFIG_O_LCD_TYPE;
	*rt = CONFIG_O_LCD_ROTA;
	#else
	*enca = 0xff;
	#endif
}

void option_set_lcd_info(uint8_t enca, uint8_t rt)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "set lcd_info");
	// 	return;
	// }

	// err = nvs_set_u8(hardware_handle, "O_LCD_TYPE", enca);
	// err |= nvs_set_u8(hardware_handle, "O_LCD_ROTA", rt ? 1 : 0);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oset_lcd_info err 0x%x", err);

	// close_partition(hardware_handle, hardware);
}

void option_get_ddmm(uint8_t *enca)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// uint8_t dmm;
	// // init default
	// *enca = ((g_device->options32) & T_DDMM) ? 1 : 0;
	// ;

	// if (open_partition(hardware, option_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "ddmm");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "O_DDMM_FLAG", (uint8_t *)&dmm);

	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oget_ddmm err 0x%x", err);
	// else
	// {
	// 	if (dmm != 255)
	// 		*enca = dmm;
	// 	if (*enca)
	// 		*enca = 1;
	// }

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_O_DDMM_FLAG
	*enca = CONFIG_O_DDMM_FLAG;
	#else
	*enca = 1;
	#endif
}

void option_set_ddmm(uint8_t enca)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "set_ddmm");
	// 	return;
	// }

	// err = nvs_set_u8(hardware_handle, "O_DDMM_FLAG", enca ? 1 : 0);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oset_ddmm err 0x%x", err);

	// close_partition(hardware_handle, hardware);
}

void option_set_lcd_out(uint32_t enca)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "set_lcd_out");
	// 	return;
	// }

	// err = nvs_set_u32(hardware_handle, "O_LCD_OUT", enca);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oset_lcd_out err 0x%x", err);

	// close_partition(hardware_handle, hardware);
}

void option_set_lcd_stop(uint32_t enca)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "set_lcd_stop");
	// 	return;
	// }

	// err = nvs_set_u32(hardware_handle, "O_LCD_STOP", enca);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oset_lcd_stop err 0x%x", err);

	// close_partition(hardware_handle, hardware);
}

void option_set_lcd_blv(int blv)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "set_lcd_blv");
	// 	return;
	// }
	// err = nvs_set_i32(hardware_handle, "O_LCD_BLV", blv);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "oset_lcd_blv err 0x%x", err);
	// close_partition(hardware_handle, hardware);
}

void option_get_lcd_out(uint32_t *enca, uint32_t *encb)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// uint32_t lout;

	// // init default lcd_out
	// //		*enca = g_device->lcd_out;
	// *enca = 0;
	// // init default lcd_stop
	// *encb = 0;

	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "lcd_out");
	// 	return;
	// }

	// err = nvs_get_u32(hardware_handle, "O_LCD_OUT", (uint32_t *)&lout);
	// if (err == ESP_ERR_NVS_NOT_FOUND)
	// 	err = nvs_set_u32(hardware_handle, "O_LCD_OUT", *enca);
	// if (err != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "oget_lcd_out err 0x%x", err);
	// }
	// else
	// {
	// 	if (lout == 255)
	// 		lout = 0; // special case
	// 	*enca = lout;
	// }

	// err = nvs_get_u32(hardware_handle, "O_LCD_STOP", (uint32_t *)&lout);
	// if (err == ESP_ERR_NVS_NOT_FOUND)
	// 	err = nvs_set_u32(hardware_handle, "O_LCD_STOP", *encb);
	// if (err != ESP_OK)
	// {
	// 	ESP_LOGI(TAG, "oget_lcd_STOP err 0x%x", err);
	// }
	// else
	// {
	// 	if (lout == 255)
	// 		lout = 0; // special case
	// 	*encb = lout;
	// }
	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_O_LCD_OUT
	*enca = CONFIG_O_LCD_OUT;
	#else
	*enca = 0;
	#endif

	#ifdef CONFIG_O_LCD_STOP
	*encb = CONFIG_O_LCD_STOP;
	#else
	*encb = 0;
	#endif
}

void option_get_lcd_blv(int *blv)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// int lout;
	// *blv = 100;
	// if (open_partition(hardware, option_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "lcd_blv");
	// 	return;
	// }

	// err = nvs_get_i32(hardware_handle, "O_LCD_BLV", (int32_t *)&lout);
	// if (err == ESP_ERR_NVS_NOT_FOUND)
	// {
	// 	err = nvs_set_i32(hardware_handle, "O_LCD_BLV", *blv);
	// 	//		option_set_lcd_blv(*blv); err = 0;
	// }
	// if (err != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "oget_lcd_blv err 0x%x", err);
	// }
	// else
	// {
	// 	ESP_LOGD(TAG, "lcd_blv3");
	// 	if (lout == 255)
	// 		lout = 100; // special case
	// 	*blv = lout;
	// }
	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_O_LCD_BLV
	*blv = CONFIG_O_LCD_BLV;
	#else
	*blv = 100;
	#endif
}

void gpio_get_ledgpio(gpio_num_t *enca)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *enca = g_device->led_gpio;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "ledgpio");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_LED_GPIO", (uint8_t *)enca);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_ledgpio err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_LED_GPIO
	*enca = CONFIG_P_LED_GPIO;
	#else
	*enca = GPIO_LED;
	#endif
}

void gpio_set_ledgpio(gpio_num_t enca)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// if (open_partition(hardware, gpio_space, NVS_READWRITE, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "set_ledgpio");
	// 	return;
	// }

	// err = nvs_set_u8(hardware_handle, "P_LED_GPIO", enca);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "gpio_set_ledgpio err 0x%x", err);

	// close_partition(hardware_handle, hardware);
}

void gpio_get_joysticks(gpio_num_t *enca, gpio_num_t *enca1)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *enca = PIN_JOY_0;
	// *enca1 = PIN_JOY_1;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "joys");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_JOY_0", (uint8_t *)enca);
	// err = nvs_get_u8(hardware_handle, "P_JOY_1", (uint8_t *)enca1);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_joysticks err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_JOY_0
	*enca = CONFIG_P_JOY_0;
	#else
	*enca = PIN_JOY_0;
	#endif
	#ifdef CONFIG_P_JOY_1
	*enca1 = CONFIG_P_JOY_1;
	#else
	*enca1 = PIN_JOY_1;
	#endif
}

// get the active level of buttons
void gpio_get_active_buttons(bool *abtn0, bool *abtn1)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// *abtn0 = 0;
	// *abtn1 = 0;
	// if (open_partition(hardware, option_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "buttons");
	// 	return;
	// }
	// err = nvs_get_u8(hardware_handle, "O_BTN0", (uint8_t *)abtn0);
	// err = nvs_get_u8(hardware_handle, "O_BTN1", (uint8_t *)abtn1);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_active_buttons err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_O_BTN0
	*abtn0 = CONFIG_O_BTN0;
	#else
	*abtn0 = 0;
	#endif

	#ifdef CONFIG_O_BTN1
	*abtn1 = CONFIG_O_BTN1;
	#else
	*abtn1 = 0;
	#endif
}

void gpio_get_buttons(gpio_num_t *enca, gpio_num_t *encb, gpio_num_t *encc, gpio_num_t *enca1, gpio_num_t *encb1, gpio_num_t *encc1)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *enca = PIN_BTN0_A;
	// *encb = PIN_BTN0_B;
	// *encc = PIN_BTN0_C;
	// *enca1 = PIN_BTN1_A;
	// *encb1 = PIN_BTN1_B;
	// *encc1 = PIN_BTN1_C;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "buttons");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_BTN0_A", (uint8_t *)enca);
	// err |= nvs_get_u8(hardware_handle, "P_BTN0_B", (uint8_t *)encb);
	// err |= nvs_get_u8(hardware_handle, "P_BTN0_C", (uint8_t *)encc);
	// err |= nvs_get_u8(hardware_handle, "P_BTN1_A", (uint8_t *)enca1);
	// err |= nvs_get_u8(hardware_handle, "P_BTN1_B", (uint8_t *)encb1);
	// err |= nvs_get_u8(hardware_handle, "P_BTN1_C", (uint8_t *)encc1);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_buttons err 0x%x", err);

	// close_partition(hardware_handle, hardware);

	#ifdef CONFIG_P_BTN0_A
	*enca = CONFIG_P_BTN0_A;
	#else
	*enca = PIN_BTN0_A;
	#endif
	
	#ifdef CONFIG_P_BTN0_B
	*encb = CONFIG_P_BTN0_B;
	#else
	*encb = PIN_BTN0_B;
	#endif

	#ifdef CONFIG_P_BTN0_C
	*encc = CONFIG_P_BTN0_C;
	#else
	*encc = PIN_BTN0_C;
	#endif

	#ifdef CONFIG_P_BTN1_A
	*enca1 = CONFIG_P_BTN1_A;
	#else
	*enca1 = PIN_BTN1_A;
	#endif
	
	#ifdef CONFIG_P_BTN1_B
	*encb1 = CONFIG_P_BTN1_B;
	#else
	*encb1 = PIN_BTN1_B;
	#endif

	#ifdef CONFIG_P_BTN1_C
	*encc1 = CONFIG_P_BTN1_C;
	#else
	*encc1 = PIN_BTN1_C;
	#endif
}

void gpio_get_encoders(gpio_num_t *enca, gpio_num_t *encb, gpio_num_t *encbtn, gpio_num_t *enca1, gpio_num_t *encb1, gpio_num_t *encbtn1)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// if (bigSram()) // default is not compatible (gpio 16 & 17)
	// {
	// 	*enca = GPIO_NONE;
	// 	*encb = GPIO_NONE;
	// 	*encbtn = GPIO_NONE;
	// 	*enca1 = GPIO_NONE;
	// 	*encb1 = GPIO_NONE;
	// 	*encbtn1 = GPIO_NONE;
	// }
	// else
	// {
	// 	*enca = PIN_ENC0_A;
	// 	*encb = PIN_ENC0_B;
	// 	*encbtn = PIN_ENC0_BTN;
	// 	*enca1 = PIN_ENC1_A;
	// 	*encb1 = PIN_ENC1_B;
	// 	*encbtn1 = PIN_ENC1_BTN;
	// }

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "enc");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_ENC0_A", (uint8_t *)enca);
	// err |= nvs_get_u8(hardware_handle, "P_ENC0_B", (uint8_t *)encb);
	// err |= nvs_get_u8(hardware_handle, "P_ENC0_BTN", (uint8_t *)encbtn);
	// err |= nvs_get_u8(hardware_handle, "P_ENC1_A", (uint8_t *)enca1);
	// err |= nvs_get_u8(hardware_handle, "P_ENC1_B", (uint8_t *)encb1);
	// err |= nvs_get_u8(hardware_handle, "P_ENC1_BTN", (uint8_t *)encbtn1);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_encoder0 err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_ENC0_A
	*enca = CONFIG_P_ENC0_A;
	#else
	*enca = PIN_ENC0_A;
	#endif

	#ifdef CONFIG_P_ENC0_B
	*encb = CONFIG_P_ENC0_B;
	#else
	*encb = PIN_ENC0_B;
	#endif

	#ifdef CONFIG_P_ENC0_BTN
	*encbtn = CONFIG_P_ENC0_BTN;
	#else
	*encbtn = PIN_ENC0_BTN;
	#endif

	#ifdef CONFIG_P_ENC1_A
	*enca1 = CONFIG_P_ENC1_A;
	#else
	*enca1 = PIN_ENC1_A;
	#endif

	#ifdef CONFIG_P_ENC1_B
	*encb1 = CONFIG_P_ENC1_B;
	#else
	*encb1 = PIN_ENC1_B;
	#endif

	#ifdef CONFIG_P_ENC1_BTN
	*encbtn1 = CONFIG_P_ENC1_BTN;
	#else
	*encbtn1 = PIN_ENC1_BTN;
	#endif
}

void gpio_get_i2c(gpio_num_t *scl, gpio_num_t *sda, gpio_num_t *rsti2c)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *scl = PIN_I2C_SCL;
	// *sda = PIN_I2C_SDA;
	// *rsti2c = PIN_I2C_RST;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "i2c");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_I2C_SCL", (uint8_t *)scl);
	// err |= nvs_get_u8(hardware_handle, "P_I2C_SDA", (uint8_t *)sda);
	// err |= nvs_get_u8(hardware_handle, "P_I2C_RST", (uint8_t *)rsti2c);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_i2c err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_I2C_SCL
	*scl = CONFIG_P_I2C_SCL;
	#else
	*scl = PIN_I2C_SCL;
	#endif

	#ifdef CONFIG_P_I2C_SDA
	*sda = CONFIG_P_I2C_SDA;
	#else
	*sda = PIN_I2C_SDA;
	#endif

	#ifdef CONFIG_P_I2C_RST
	*rsti2c = CONFIG_P_I2C_RST;
	#else
	*rsti2c = PIN_I2C_RST;
	#endif
}

void gpio_get_spi_lcd(gpio_num_t *cs, gpio_num_t *a0, gpio_num_t *rstlcd)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *cs = PIN_LCD_CS;
	// *a0 = PIN_LCD_A0;
	// *rstlcd = PIN_LCD_RST;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "spi_lcd");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_LCD_CS", (uint8_t *)cs);
	// err |= nvs_get_u8(hardware_handle, "P_LCD_A0", (uint8_t *)a0);
	// err |= nvs_get_u8(hardware_handle, "P_LCD_RST", (uint8_t *)rstlcd);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_spi_lcd err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_LCD_CS
	*cs = CONFIG_P_LCD_CS;
	#else
	*cs = PIN_LCD_CS;
	#endif

	#ifdef CONFIG_P_LCD_A0
	*a0 = CONFIG_P_LCD_A0;
	#else
	*a0 = PIN_LCD_A0;
	#endif

	#ifdef CONFIG_P_LCD_RST
	*rstlcd = CONFIG_P_LCD_RST;
	#else
	*rstlcd = PIN_LCD_RST;
	#endif
}

void gpio_get_ir_signal(gpio_num_t *ir)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *ir = PIN_IR_SIGNAL;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "ir");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_IR_SIGNAL", (uint8_t *)ir);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_ir_signal err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_IR_SIGNAL
	*ir = CONFIG_P_IR_SIGNAL;
	#else
	*ir = PIN_IR_SIGNAL;
	#endif
}

void gpio_get_adc(adc1_channel_t *channel, adc1_channel_t *chanbatt)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default keyboard gpio
	// *channel = PIN_ADC;
	// *channel = gpioToChannel(*channel);
	// // battery gpio
	// *chanbatt = GPIO_NONE;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "adc");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_ADC_KBD", (uint8_t *)channel);
	// if (err != ESP_OK)
	// 	ESP_LOGW(TAG, "g_get_adc err 0x%x", err);
	// else
	// 	*channel = gpioToChannel(*channel);

	// err = nvs_get_u8(hardware_handle, "P_ADC_BAT", (uint8_t *)chanbatt);
	// if (err != ESP_OK)
	// 	ESP_LOGW(TAG, "g_get_adc err 0x%x", err);
	// else
	// 	*chanbatt = gpioToChannel(*chanbatt);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_ADC_KBD
	*channel = gpioToChannel(CONFIG_P_ADC_KBD);
	#else
	*channel = gpioToChannel(PIN_ADC);
	#endif

	#ifdef CONFIG_P_ADC_BAT
	*chanbatt = gpioToChannel(CONFIG_P_ADC_BAT);
	#else
	*chanbatt = gpioToChannel(GPIO_NONE);
	#endif
}

void gpio_get_i2s(gpio_num_t *lrck, gpio_num_t *bclk, gpio_num_t *i2sdata)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *lrck = PIN_I2S_LRCK;
	// *bclk = PIN_I2S_BCLK;
	// *i2sdata = PIN_I2S_DATA;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "i2s");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_I2S_LRCK", (uint8_t *)lrck);
	// err |= nvs_get_u8(hardware_handle, "P_I2S_BCLK", (uint8_t *)bclk);
	// err |= nvs_get_u8(hardware_handle, "P_I2S_DATA", (uint8_t *)i2sdata);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_i2s err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_I2S_LRCK
	*lrck = CONFIG_P_I2S_LRCK;
	#else
	*lrck = PIN_I2S_LRCK;
	#endif

	#ifdef CONFIG_P_I2S_BCLK
	*bclk = CONFIG_P_I2S_BCLK;
	#else
	*bclk = PIN_I2S_BCLK;
	#endif

	#ifdef CONFIG_P_I2S_DATA
	*i2sdata = CONFIG_P_I2S_DATA;
	#else
	*i2sdata = PIN_I2S_DATA;
	#endif
}

void gpio_get_lcd_backlightl(gpio_num_t *lcdb)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *lcdb = PIN_LCD_BACKLIGHT;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "lcdback");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_BACKLIGHT", (uint8_t *)lcdb);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_lcd_backlightl err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_BACKLIGHT
	*lcdb = CONFIG_P_BACKLIGHT;
	#else
	*lcdb = PIN_LCD_BACKLIGHT;
	#endif
}

void gpio_get_touch(gpio_num_t *cs)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;
	// // init default
	// *cs = PIN_TOUCH_CS;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "touch");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_TOUCH_CS", (uint8_t *)cs);
	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "g_get_touch err 0x%x", err);

	// close_partition(hardware_handle, hardware);
	#ifdef CONFIG_P_TOUCH_CS
	*cs = CONFIG_P_TOUCH_CS;
	#else
	*cs = PIN_TOUCH_CS;
	#endif
}

uint8_t gpioToChannel(uint8_t gpio)
{
	if (gpio == GPIO_NONE)
		return GPIO_NONE;
	if (gpio >= 38)
		return (gpio - 36);
	else
		return (gpio - 28);
}

// bool gpio_get_ir_key(nvs_handle handle, const char *key, uint32_t *out_value1, uint32_t *out_value2)
// {
// 	// init default
// 	bool ret = false;
// 	*out_value1 = 0;
// 	*out_value2 = 0;
// 	size_t required_size;
// 	nvs_get_str(handle, key, NULL, &required_size);
// 	if (required_size > 1)
// 	{
// 		char *string = malloc(required_size);
// 		nvs_get_str(handle, key, string, &required_size);
// 		sscanf(string, "%x %x", out_value1, out_value2);
// 		//		ESP_LOGV(TAG,"String \"%s\"\n Required size: %d",string,required_size);
// 		free(string);
// 		ret = true;
// 	}

// 	ESP_LOGV(TAG, "Key: %s, value1: %x, value2: %x, ret: %d", key, *out_value1, *out_value2, ret);

// 	return ret;
// }

/** Get the GPIO (P_SLEEP) for Deep Sleep power saving mode. */
/*  Get the level (P_LEVEL_SLEEP) of pin (P_SLEEP) that triggers power saving. */
void gpio_get_pinSleep(gpio_num_t *pin, bool *aLevel)
{
	// esp_err_t err;
	// nvs_handle hardware_handle;

	// // init defaults from gpio.h
	// *pin = PIN_SLEEP;
	// *aLevel = LEVEL_SLEEP;

	// if (open_partition(hardware, gpio_space, NVS_READONLY, &hardware_handle) != ESP_OK)
	// {
	// 	ESP_LOGD(TAG, "buttons");
	// 	return;
	// }

	// err = nvs_get_u8(hardware_handle, "P_SLEEP", (uint8_t *)pin);
	// err |= nvs_get_u8(hardware_handle, "P_LEVEL_SLEEP", (uint8_t *)aLevel);

	// if (err != ESP_OK)
	// 	ESP_LOGD(TAG, "gpio_get_pinDeepSleep err 0x%x", err);

	// close_partition(hardware_handle, hardware);

	#ifdef CONFIG_P_SLEEP
	*pin = CONFIG_P_SLEEP;
	#else
	*pin = PIN_SLEEP;
	#endif

	#ifdef CONFIG_P_LEVEL_SLEEP
	*aLevel = CONFIG_P_LEVEL_SLEEP;
	#else
	*aLevel = LEVEL_SLEEP;
	#endif
}
