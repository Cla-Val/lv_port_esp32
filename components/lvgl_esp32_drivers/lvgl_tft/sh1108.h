/**
 * @file lv_templ.h
 *
 */

#ifndef SH1108_H
#define SH1108_H

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
#define DISP_BUF_SIZE   ((CONFIG_LVGL_DISPLAY_WIDTH*CONFIG_LVGL_DISPLAY_HEIGHT))
										//-Because 8 bits per pixel ...
#define SH1108_DC   CONFIG_LVGL_DISP_PIN_DC
#define SH1108_RST  CONFIG_LVGL_DISP_PIN_RST

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void sh1108_init(void);
void sh1108_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
void sh1108_rounder(struct _disp_drv_t * disp_drv, lv_area_t *area);
void sh1108_set_px_cb(struct _disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa);
void sh1108_sleep_in(void);
void sh1108_sleep_out(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*SH1108_H*/
