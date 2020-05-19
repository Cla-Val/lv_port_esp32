/**
 * @file lv_templ.h
 *
 */

#ifndef ILI9341_H
#define ILI9341_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#include "lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/
#define DISP_BUF_SIZE (LV_HOR_RES_MAX * 40)
#define ILI9341_DC   CONFIG_LVGL_DISP_PIN_DC
#define ILI9341_RST  CONFIG_LVGL_DISP_PIN_RST
#define ILI9341_BCKL CONFIG_LVGL_DISP_PIN_BCKL

#define ILI9341_ENABLE_BACKLIGHT_CONTROL CONFIG_LVGL_ENABLE_BACKLIGHT_CONTROL

#if CONFIG_LVGL_BACKLIGHT_ACTIVE_LVL
  #define ILI9341_BCKL_ACTIVE_LVL 1
#else
  #define ILI9341_BCKL_ACTIVE_LVL 0
#endif

#if (CONFIG_LVGL_DISPLAY_ORIENTATION_PORTRAIT)
    #define ILI9341_DISPLAY_ORIENTATION	TFT_ORIENTATION_PORTRAIT
#elif (CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE)
    #define ILI9341_DISPLAY_ORIENTATION	TFT_ORIENTATION_LANDSCAPE
#endif

#define ILI9341_INVERT_DISPLAY CONFIG_LVGL_INVERT_DISPLAY
#define ILI9341_INVERT_COLORS CONFIG_LVGL_INVERT_COLORS

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ili9341_init(void);
void ili9341_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
void ili9341_enable_backlight(bool backlight);
void ili9341_sleep_in(void);
void ili9341_sleep_out(void);

/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*ILI9341_H*/
