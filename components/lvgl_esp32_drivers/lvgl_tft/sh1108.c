
/*********************
 *      INCLUDES
 *********************/
#include "sh1108.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "SH1108"

/**********************
 *      TYPEDEFS
 **********************/

typedef struct
	{
	uint8_t
		cmd[4],
		nbr_bytes;
	} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void sh1108_send_cmd(const uint8_t *cmd, uint8_t length);
static void sh1108_send_data(void * data, uint16_t length);
static void sh1108_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void sh1108_init(void)
	{
	static const lcd_init_cmd_t
		OLED_init_cmds[] =
				{
				{{0xAE},1},					//-Display OFF
				{{0x20},1},					//-Memory addressing mode
				{{0x81,0xD0},2},			//-Contrast
				{{0xA0},1},					//-Segment remapping
				{{0xA6},1},					//-Normal display
				{{0xA9,0x02},2},			//-Select resolution 128 x 160
				{{0xAD,0x80},2},			//-External Vpp
				{{0xC0},1},					//-Common scan direction
				{{0xD5,0xF1},2},			//-Divide ratio 100 Hz
				{{0xD9,0x13},2},			//-Discharge/precharge period
				{{0xDB,0x2B},2},			//-Vcomh voltage (Vpp x 0.687)
				{{0xDC,0x35},2},			//-VSEGM deselect level
				{{0x30},1},					//-Discharge VSL level = 0V
				{{0xAF},1},					//-Display ON
				{{},0}						//-End of command list
				};

	printf("sh1108_init: W:%d, H:%d\n",CONFIG_LVGL_DISPLAY_WIDTH,CONFIG_LVGL_DISPLAY_HEIGHT);
    gpio_config_t
		io_conf_out =
			{
			.intr_type = GPIO_PIN_INTR_DISABLE,
										//-No interrupts
			.mode = GPIO_MODE_OUTPUT,	//-Mode is GPIO OUTPUT
			.pin_bit_mask = ((1ULL << SH1108_DC) | (1ULL << SH1108_RST)),
										//-Bitmap of GPIOs to configure
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
										//-No pullups (not an input)
			.pull_up_en = GPIO_PULLUP_DISABLE
			};							// ... and no pulldown either.
    if (gpio_config(&io_conf_out) != ESP_OK)
		printf("error configuring outputs \n");
	gpio_set_direction(SH1108_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(SH1108_RST, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(SH1108_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(SH1108_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

printf("Initialising controller\n");
	//Send all the commands
/*
	uint16_t cmd = 0;
	while (OLED_init_cmds[cmd].databytes!=0xff) {
		sh1108_send_cmd(init_cmds[cmd].cmd);
		sh1108_send_data(init_cmds[cmd].data, init_cmds[cmd].databytes&0x1F);
		if (init_cmds[cmd].databytes & 0x80) {
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
		}
*/
//    spi_trans.user=(void*)A0_IS_COMMAND;
    for (int i = 0; OLED_init_cmds[i].nbr_bytes; i++)
 		sh1108_send_cmd(OLED_init_cmds[i].cmd,OLED_init_cmds[i].nbr_bytes);
	}

static bool
	dump_pixels = true;
static int
	pixel_count = 0;

void sh1108_set_px_cb(struct _disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa)
	{
	/* buf_w will be ignored, the configured CONFIG_LVGL_DISPLAY_HEIGHT and _WIDTH,
	and CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE and _PORTRAIT will be used. */
	uint16_t
		byte_index = 0;
	uint8_t
		bit_index = 0;

//	#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE
//	byte_index = y + (( x>>3 ) * CONFIG_LVGL_DISPLAY_HEIGHT);
//	bit_index  = x & 0x7;
//	#elif defined CONFIG_LVGL_DISPLAY_ORIENTATION_PORTRAIT
	byte_index = x + (( y>>3 ) * CONFIG_LVGL_DISPLAY_WIDTH);
	bit_index  = y & 0x7;
//	#endif

//	if ( color.full == 0 )
	if (color.full)
		{
		if (dump_pixels)
			printf("Set pixel @%d:%d (byte/bit %d:%d) %p\n",x,y,byte_index,bit_index,buf);
		BIT_SET(buf[byte_index], bit_index);
		}
	else
		{
		if (dump_pixels)
			printf("Clr pixel @%d:%d (byte/bit %d:%d) %p\n",x,y,byte_index,bit_index,buf);
		BIT_CLEAR(buf[byte_index], bit_index);
		}
	if (dump_pixels)
		{
		if ((++pixel_count) >= 10)
			{
			pixel_count=0;
			dump_pixels=false;
			}
		}
	}

void sh1108_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t
		columnLow = (area->x1 + 16) & 0x0F;
	uint8_t
		columnHigh = ((area->x1 + 16) >> 4) & 0x0F;
										//-There's an offset of 16 to the column
										// (COM) we send out, 'cos when we're in
										// 128 COM * 160 SEG mode that's the way
										// it's mapped ...
    uint8_t
		row1 = 0,
		row2 = 0;
    uint32_t
		size = 0;
    void
		*ptr;

	dump_pixels=true;
/**/
	printf("Flushing buffer @%p, %d:%d - %d:%d\n",
			color_map,
			area->x1,
			area->y1,
			area->x2,
			area->y2);
/**/
//#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE
//    row1 = area->x1>>3;
//    row2 = area->x2>>3;
//#else
    row1 = area->y1>>3;
    row2 = area->y2>>3;
//#endif
    for(int i = row1; i < row2+1; i++)
		{
		uint8_t
			cmd[4];
		cmd[0]=0x10 | columnHigh;
		cmd[1]=0x00 | columnLow;
		cmd[2]=0xB0;
		cmd[3]=i;
	    sh1108_send_cmd(cmd,4);
	    size = area->x2 - area->x1 + 1;
/*
#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE
        ptr = color_map + i * CONFIG_LVGL_DISPLAY_HEIGHT;
#else
        ptr = color_map + i * CONFIG_LVGL_DISPLAY_WIDTH;
#endif
*/
        ptr = color_map + i * 128;
        sh1108_send_color( (void *) ptr, size);
		}
}

void sh1108_rounder(struct _disp_drv_t * disp_drv, lv_area_t *area)
{
    area->y1 = (area->y1 & (~0x7));
    area->y2 = (area->y2 & (~0x7)) + 7;
}

void sh1108_sleep_in()
	{
	uint8_t
		cmd = 0xAE;

	sh1108_send_cmd(&cmd,1);
	}

void sh1108_sleep_out()
	{
	uint8_t
		cmd = 0xAF;

	sh1108_send_cmd(&cmd,1);
	}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void sh1108_send_cmd(const uint8_t *cmd, uint8_t length)
	{
    while(disp_spi_is_busy()) {}
    gpio_set_level(SH1108_DC, 0);	 /*Command mode*/
    disp_spi_send_cmd(cmd, length);
	}

static void sh1108_send_data(void * data, uint16_t length)
{
    while(disp_spi_is_busy()) {}
    gpio_set_level(SH1108_DC, 1);	 /*Data mode*/
    disp_spi_send_data(data, length);
}

static void sh1108_send_color(void * data, uint16_t length)
{
    while(disp_spi_is_busy()) {}
	printf("    sh1108_send_color punting off %d bytes from %p\n",length,data);
    gpio_set_level(SH1108_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}
