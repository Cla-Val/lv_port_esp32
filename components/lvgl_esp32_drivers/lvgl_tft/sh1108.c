
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
    // Use Double Bytes Commands if necessary, but not Command+Data
    // Initialization taken from https://github.com/nopnop2002/esp-idf-m5stick
/*
	lcd_init_cmd_t init_cmds[]={
    	{0xAE, {0}, 0},	// Turn display off
    	{0xDC, {0}, 0},	// Set display start line
    	{0x00, {0}, 0},	// ...value
    	{0x81, {0}, 0},	// Set display contrast
    	{0x2F, {0}, 0},	// ...value
    	{0x20, {0}, 0},	// Set memory mode
    	{0xA0, {0}, 0},	// Non-rotated display  
#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE			
    	{0xC8, {0}, 0},	// flipped vertical
#elif defined CONFIG_LVGL_DISPLAY_ORIENTATION_PORTRAIT
    	{0xC7, {0}, 0},	// flipped vertical
#endif
    	{0xA8, {0}, 0},	// Set multiplex ratio
    	{0x7F, {0}, 0},	// ...value
    	{0xD3, {0}, 0},	// Set display offset to zero
    	{0x60, {0}, 0},	// ...value
    	{0xD5, {0}, 0},	// Set display clock divider
    	{0x51, {0}, 0},	// ...value
    	{0xD9, {0}, 0},	// Set pre-charge
    	{0x22, {0}, 0},	// ...value
    	{0xDB, {0}, 0},	// Set com detect
    	{0x35, {0}, 0},	// ...value
    	{0xB0, {0}, 0},	// Set page address
    	{0xDA, {0}, 0},	// Set com pins
    	{0x12, {0}, 0},	// ...value
    	{0xA4, {0}, 0},	// output ram to display
#if defined CONFIG_LVGL_INVERT_DISPLAY
    	{0xA7, {0}, 0},	// inverted display
#else
    	{0xA6, {0}, 0},	// Non-inverted display
#endif 
    	{0xAF, {0}, 0},	// Turn display on
        {0, {0}, 0xff},
	};
*/
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

	//Initialize non-SPI GPIOs
	gpio_set_direction(SH1108_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(SH1108_RST, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(SH1108_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(SH1108_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

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

void sh1108_set_px_cb(struct _disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa) 
{
	/* buf_w will be ignored, the configured CONFIG_LVGL_DISPLAY_HEIGHT and _WIDTH,
	   and CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE and _PORTRAIT will be used. */ 		
    uint16_t byte_index = 0;
    uint8_t  bit_index = 0;

#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE			
	byte_index = y + (( x>>3 ) * CONFIG_LVGL_DISPLAY_HEIGHT);
	bit_index  = x & 0x7;
#elif defined CONFIG_LVGL_DISPLAY_ORIENTATION_PORTRAIT
    byte_index = x + (( y>>3 ) * CONFIG_LVGL_DISPLAY_WIDTH);
    bit_index  = y & 0x7;
#endif

// #ifndef CONFIG_LVGL_INVERT_DISPLAY
    if ( color.full == 0 ) {
// #else
    // if ( color.full != 0 ) {
// #endif
        BIT_SET(buf[byte_index], bit_index);
    } else {
        BIT_CLEAR(buf[byte_index], bit_index);
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
    uint8_t row1 = 0, row2 = 0;
    uint32_t size = 0;
    void *ptr;

#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE		
    row1 = area->x1>>3;
    row2 = area->x2>>3;
#else 
    row1 = area->y1>>3;
    row2 = area->y2>>3;
#endif
    for(int i = row1; i < row2+1; i++)
		{
		uint8_t
			cmd[4];
/*			
		cmd[0]=0x10 | columnHigh;
	    sh1108_send_cmd(cmd,1);
		cmd[0]=0x00 | columnLow;
	    sh1108_send_cmd(cmd,1);
		cmd[0]=0xB0;
		cmd[1]=i;
	    sh1108_send_cmd(cmd,2); 
*/
		cmd[0]=0x10 | columnHigh;
		cmd[1]=0x00 | columnLow;
		cmd[2]=0xB0;
		cmd[3]=i;
	    sh1108_send_cmd(cmd,4); 
	    size = area->y2 - area->y1 + 1;
#if defined CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE		
        ptr = color_map + i * CONFIG_LVGL_DISPLAY_HEIGHT;
#else 
        ptr = color_map + i * CONFIG_LVGL_DISPLAY_WIDTH;
#endif
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
    gpio_set_level(SH1108_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}
