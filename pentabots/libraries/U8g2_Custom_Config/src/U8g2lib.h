#error "If you see this message, the custom U8g2lib.h is being used!"

#ifndef U8G2LIB_H
#define U8G2LIB_H

// --- U8g2 Project Specific Configuration ---
// This file is used to disable unused U8g2 features to save program memory (Flash).

// 1. We are using an SSD1306 display. Enable the drivers for it.
#define U8G2_HAVE_CONTROLLER_SSD1306

// 2. We are using hardware I2C. Enable the necessary communication functions.
//    By defining this, we can disable all the SPI code.
#define U8G2_HAVE_HW_I2C

// 3. We are using the page buffer mode. Disable the full buffer mode.
#define U8G2_WITH_PAGED_MODE

// 4. Select ONLY the fonts you are using in your project.
//    Fonts are one of the biggest consumers of Flash memory.
//    Your code uses: u8g2_font_ncenB08_tr and u8g2_font_ncenB14_tr
#define U8G2_FONT_SECTION ".progmem.u8g2_font"
#include <u8g2_fonts.h>
U8G2_FONT_TABLE_BEGIN(u8g2_font_table)
  U8G2_FONT_TABLE_ENTRY(u8g2_font_ncenB14_tr)
U8G2_FONT_TABLE_END(u8g2_font_table)u8g2_font_ncenB14_tr

// 5. Finally, include the main U8g2 header file to pull in the configured code.
#include <clib/u8g2.h>

#endif // U8G2LIB_H