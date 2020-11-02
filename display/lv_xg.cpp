#include "lvgl.h"
#include "XGLCD.h"

#if defined(XG_LCD_USE_FT5206)
  #include <FT5206.h>

              
  #define CTP_INT           15    // touch data ready for read from FT5206 touch controller
  #define FT5206_RST        16

  uint8_t registers[FT5206_REGISTERS];
  uint16_t new_coordinates[5][2];
  uint8_t current_touches = 0;


  FT5206 cts = FT5206(CTP_INT);


  bool my_tp_read(lv_indev_drv_t * indev, lv_indev_data_t *data) {
    if (cts.touched()){
      cts.getTSregisters(registers);
      current_touches = cts.getTScoordinates(new_coordinates, registers);
      if (current_touches < 1) {
        data->state = LV_INDEV_STATE_REL;
        return false;
      } 
      data->point.x = new_coordinates[0][0];
      data->point.y = new_coordinates[0][1];
      data->state = LV_INDEV_STATE_PR;
    }
    else
    data->state = LV_INDEV_STATE_REL;
    return false;
  }  
#elif defined(XG_LCD_USE_GSL1680)
  #include <GSL1680.h>

  #define CTP_WAKE    14
  #define CTP_INTRPT  32

  GSL1680 TS = GSL1680();

  bool my_tp_read(lv_indev_drv_t * indev, lv_indev_data_t *data) {
    data->state = LV_INDEV_STATE_REL;
    int NBFinger = TS.dataread();
    if (NBFinger > 0) {
      data->point.x = TS.readFingerX(0);
      data->point.y = TS.readFingerY(0);
      data->state = LV_INDEV_STATE_PR;
    }
    return false;
  }
#endif

XGLCD _tft = XGLCD();

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  _tft.drawBitmap(area->x1, area->y1, area->x2, area->y2, (uint16_t *)color_p);                    // copy 'color_array' to the specifed coordinates
  lv_disp_flush_ready(disp);                                                           // Tell the flushing is ready
}

void lv_xg_init() {

  lv_init();                                                                  // Init the LittleVGL library (not C++ library)
  
  _tft.begin();     
  delay(300);

#if defined(XG_LCD_USE_FT5206)

  #if defined(FT5206_RST)

    pinMode(FT5206_RST, OUTPUT);
    digitalWrite(FT5206_RST, HIGH);
    delay(10);
    digitalWrite(FT5206_RST, LOW);
    delay(220);
    digitalWrite(FT5206_RST, HIGH);
    delay(300);
  #endif

  cts.begin();
  cts.setTouchLimit(1);

#elif defined(XG_LCD_USE_GSL1680)

  TS.begin(CTP_WAKE, CTP_INTRPT);

#endif

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);                                                           // Start the LCD

  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = _tft.width();
  disp_drv.ver_res = _tft.height();
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_t indev_drv;                                                   // Link the touchscreen to the LittleVGL library
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_tp_read;
  lv_indev_drv_register(&indev_drv);

}
