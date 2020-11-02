#ifndef _ERDisplay_H_
#define _ERDisplay_H_

/*  ----------  STUFF YOU CAN FIDDLE WITH  ----------  */

// If you do not want to use LCD auto-detection, manual enable one (and only one) of below LCD types
// #define ER_LCD_50             // 5.0" 800x480 LCD
// #define ER_LCD_56             // 5.6" 640x480 LCD
// #define ER_LCD_90             // 7.0" 800x480 LCD
// #define ER_LCD_90             // 9.0" 800x480 LCD

// If your screen doesn't feature the FT5206 (for example, the East Rising displays from BuyDisplay.com) comment out this line and uncomment the line below for the GSL1680 driver

//#define ER_LCD_USE_FT5206
#define ER_LCD_USE_GSL1680

/*  ----------  STUFF YOU SHOULDN'T FIDDLE WITH  ----------  */




#include <stdio.h>
#if defined(_FORCE_PROGMEM__)                                                   // Disable PROGMEM which is not needed on 32-bit CPU's with a lot of RAM
    #undef _FORCE_PROGMEM__
#endif
#define __PRGMTAG_

#if !defined(ER_LCD_50) && !defined(ER_LCD_56) && !defined(ER_LCD_90)
    #define ER_LCD_AUTO
#endif

#if defined(ARDUINO_ARCH_SAMD)                                                  // Arduino MKR family (and all other SAMD modules, but only the MKR will work)
    #define ER_CPU_MKR
#elif defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_FEATHER_ESP32)              // ESP32 Development Kit C / Pycom modules (without Python of course)
// Pycom modules (the latest only) have 8Mbit Flash and 4Mbit PSRAM
    #define ER_CPU_ESP32
#elif defined(PLATFORM_ID)                                                      // Particle Family, this library should be used in the Particle IDE not in the Arduino IDE
// PLATFORM_PHOTON_PRODUCTION
// PLATFORM_ELECTRON_PRODUCTION
// PLATFORM_ARGON -> NRF5 based
// PLATFORM_BORON -> NRF5 based
// PLATFORM_XENON
    #define ER_CPU_PARTICLE
#elif defined(FANSTEL)                                                          // Fanstel module
//#elif defined(ARDUINO_ARCH_NRF5)                                              // Nordic NRF5x family
    #define ER_CPU_FANSTEL
#else                                                                           // All other boards presume the Arduino headers are used, 32-bit boards work better, generic SPI commands are used
    #define ER_CPU_ARDUINO
#endif

#if defined(ER_CPU_MKR)                                                         // Arduino MKR family

    #include <Arduino.h>
    #include <pins_arduino.h>
    #include <SPI.h>

    #define SPI_SPEED_WRITE                 4                                   // 12 MHz which is the max for the __SMAD21G18A__ CPU (bug in chip cause 2 should be possible)
    #define SPI_SPEED_READ                  16                                  // 3 MHz @ 48MHz CPU clock
    #define SPI_SPEED_SLOW                  32                                  // must be slower then 2 MHz before PLL is configured -> 1.5MHz @ 48MHz CPU clock
    #define SPI_MODE_LCD                    SPI_MODE3

    //#define _spiwrite(c)                    _writeMKRSPI(c)
    //#define _spiwrite16(d)                  _writeMKRSPI16(d)
    #define _spiwrite(c)                    {SERCOM1->SPI.DATA.bit.DATA = c;while(SERCOM1->SPI.INTFLAG.bit.TXC == 0);}
    #define _spiwrite16(c)                  {SERCOM1->SPI.DATA.bit.DATA = c>>8;SERCOM1->SPI.DATA.bit.DATA = c & 0xff;while(SERCOM1->SPI.INTFLAG.bit.TXC == 0);}
    #define _spiwritedma(wbuf, length)      {uint16_t i;for (i=0; i<length; i++) {while(SERCOM1->SPI.INTFLAG.bit.DRE == 0);SERCOM1->SPI.DATA.bit.DATA = buf[i];}while(SERCOM1->SPI.INTFLAG.bit.TXC == 0);}

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 SPI.setClockDivider(s)

    #define _spiCSLow                       PORT->Group[g_APinDescription[ER_PIN_LCD_CS].ulPort].OUTCLR.reg = (1UL << g_APinDescription[ER_PIN_LCD_CS].ulPin)
    #define _spiCSHigh                      PORT->Group[g_APinDescription[ER_PIN_LCD_CS].ulPort].OUTSET.reg = (1UL << g_APinDescription[ER_PIN_LCD_CS].ulPin)

    //LED = 6 (LED_BUILTIN)
    #define ER_PIN_BL (0u)
    #define ER_PIN_DC (1u)
    #define ER_PIN_LCD_RESET (4u)
    #define ER_PIN_SD_CS (3u)
    #define ER_PIN_TP_CS PIN_A3
    #define ER_PIN_TP_IRQ PIN_A2
    #define ER_PIN_LCD_CS (5u)
    #define ER_PIN_MOSI (8u)
    #define ER_PIN_MISO (10u)
    #define ER_PIN_SCK (9u)

    #define ER_PIN_MISO2 PIN_A4
    #define ER_PIN_MOSI2 PIN_A5
    #define ER_PIN_SCK2 PIN_A6
    #define ER_PIN_CS2 (2u)
    #define ER_PIN_D6 (6u)                                                      // also LED
    #define ER_PIN_D7 (7u)
    #define ER_PIN_SDA (11u)
    #define ER_PIN_SCL (12u)
    #define ER_PIN_RX (13u)
    #define ER_PIN_TX (14u)

    #define ER_PIN_A0 A4
    #define ER_PIN_A1 A5
    #define ER_PIN_A2 A6

#elif defined(ER_CPU_ESP32)                                                     // ESP32 Development Kit C / Pycom modules (without Python of course)

    #include <Arduino.h>
    #include <SPI.h>
    #include "soc/spi_struct.h"
    #include "driver/spi_common.h"
    #include "soc/dport_reg.h"

    #define SPI_SPEED_WRITE                 12000000
    #define SPI_SPEED_READ                  4000000
    #define SPI_SPEED_SLOW                  2000000
    #define SPI_MODE_LCD                    SPI_MODE3

    #define _spiwrite(c)                    {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 7;\
                                            dev->data_buf[0] = c;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    #define _spiwrite16(d)                  {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 15;\
                                            dev->data_buf[0] = ((d) >> 8) | ((d) << 8);\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    #define _spiwrite24(r,d)                {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 23;\
                                            dev->data_buf[0] = ((d) && 0xff00) | ((d) << 16) | r;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    #define _spixwritedma(c, wbuf, len)     {\
                                            dev->dma_conf.val |= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;\
                                            dev->dma_out_link.start=0;\
                                            dev->dma_conf.val &= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);\
                                            uint32_t n=0;\
                                            uint8_t cmd = c;\
                                            dmadesc[n].size = 1;\
                                            dmadesc[n].length = 1;\
                                            dmadesc[n].eof = 0;\
                                            dmadesc[n].buf = (uint8_t *)&cmd;\
                                            dmadesc[n].qe.stqe_next = &dmadesc[n+1];\
                                            n++;\
                                            len *=2;\
                                            dev->mosi_dlen.usr_mosi_dbitlen = len*8-1+8;\
                                            uint32_t dmachunklen;\
                                            while (len) {\
                                                dmachunklen = len;\
                                                if (dmachunklen > SPI_MAX_DMA_LEN) dmachunklen = SPI_MAX_DMA_LEN;\
                                                dmadesc[n].size = dmachunklen;\
                                                dmadesc[n].length = dmachunklen;\
                                                dmadesc[n].eof = 0;\
                                                dmadesc[n].buf = (uint8_t *)wbuf;\
                                                dmadesc[n].qe.stqe_next = &dmadesc[n+1];\
                                                len -= dmachunklen;\
                                                wbuf += dmachunklen/2;\
                                                n++;\
                                            }\
                                            dmadesc[n - 1].eof = 1;\
                                            dmadesc[n - 1].qe.stqe_next = NULL;\
                                            dev->dma_out_link.start=1;\
                                            dev->user.usr_miso=0;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    #define _spixread(x, r)                 {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 15;\
                                            dev->data_buf[0] = x;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            r = dev->data_buf[0] >> 8;\
                                            }
    #define _spixbread(x, r, b)             {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = b-1+8;\
                                            dev->miso_dlen.usr_miso_dbitlen = b-1+8;\
                                            dev->data_buf[0] = x;\
                                            dev->data_buf[1] = 0x00;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            r = (dev->data_buf[0]>>8) + (dev->data_buf[1]<<24);\
                                            dev->miso_dlen.usr_miso_dbitlen = 15;\
                                            }
#if defined(ARDUINO_FEATHER_ESP32) //the Adafruit Huzzah ESP32 board has an odd pin definition
    #define _spibegin()                     {SPI.begin(5, 19, 18, 4);}
#else
    #define _spibegin()                     {SPI.begin();}
#endif
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 SPI.setFrequency(s)
    #define _spisetBitLen                   {\
                                            SPI.setHwCs(true);\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 15;\
                                            dev->miso_dlen.usr_miso_dbitlen = 15;\
                                            dev->user2.val = 0;\
                                            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);\
                                            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);\
                                            DPORT_SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, 3, 2, 4);\
                                            dev->dma_conf.out_data_burst_en = 1;\
                                            dev->dma_conf.indscr_burst_en = 1;\
                                            dev->dma_conf.outdscr_burst_en = 1;\
                                            dev->dma_in_link.addr = 0;\
                                            dev->dma_in_link.start = 0;\
                                            dev->dma_out_link.addr = (uint32_t)(&dmadesc[0]) & 0xFFFFF;\
                                            for (uint32_t n=0; n<32; n++) {\
                                                dmadesc[n].offset = 0;\
                                                dmadesc[n].sosf = 0;\
                                                dmadesc[n].owner = 1;\
                                            }\
                                            }

    #define _spiCSLow
    #define _spiCSHigh
    #define ER_PIN_LCD_RESET 17 //34
    #define ER_PIN_SD_CS 4
    #define ER_PIN_LCD_CS SS
    
 

#elif defined(ER_CPU_PARTICLE)                                                  // Particle Family, this library should be used in the Particle IDE not in the Arduino IDE
    #include <Arduino.h>
    #define SPI_SPEED_WRITE                 SPI_CLOCK_DIV4                      // 15 MHz (divider /6 does not work) (SPI base clock = CPU clock / 2)
    #define SPI_SPEED_READ                  SPI_CLOCK_DIV16                     // 3.75MHz / Theoretical: 7.5 MHz @ 120MHz CPU clock
    #define SPI_SPEED_SLOW                  SPI_CLOCK_DIV32                     // must be slower then 2 MHz before PLL is configured -> 1.82MHz @ 120MHz CPU clock
    #define SPI_MODE_LCD                    SPI_MODE3

    #define _spiwrite(c)                    SPI.transfer(c)
    #define _spiwrite16(d)                  {SPI.transfer(d >> 8);SPI.transfer(d & 0xFF);}
    #define _spiwritedma(wbuf, length)      SPI.transfer(wbuf, NULL, length, NULL);

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 SPI.setClockDivider(s)

    #define _spiCSLow                       pinResetFast(ER_PIN_LCD_CS)
    #define _spiCSHigh                      pinSetFast(ER_PIN_LCD_CS)

    #define LED_BUILTIN D7                                                      //Shared with BL
    #define ER_PIN_BL D7                                                        //D7 shared with LED
    #define ER_PIN_DC A7                                                        //D15 = 17 / Also WKP on Duo & Photon
    #define ER_PIN_LCD_RESET D6
    #define ER_PIN_SD_CS A6                                                     //D14 = 16 = DAC1 on Photon
    #define ER_PIN_TP_CS A0                                                     //D8 = 10
    #define ER_PIN_TP_IRQ A1                                                    //D9 = 11
    #define ER_PIN_LCD_CS A2                                                    //D10 = 12 = DAC1 on Duo
    #define ER_PIN_MOSI MOSI                                                    //D13 = 15
    #define ER_PIN_MISO MISO                                                    //D12 = 14
    #define ER_PIN_SCK SCK                                                      //D11 = 13 = DAC2

    #define ER_PIN_MISO2 D3
    #define ER_PIN_MOSI2 D2
    #define ER_PIN_SCK2 D4
    #define ER_PIN_CS2 D5
    #define ER_PIN_D6
    #define ER_PIN_D7
    #define ER_PIN_SDA SDA                                                      //D0
    #define ER_PIN_SCL SCL                                                      //D1
    #define ER_PIN_RX RX                                                        //D16 = 18
    #define ER_PIN_TX TX                                                        //D17 = 19

    #define ER_PIN_A0 A0                                                        // TP CS
    #define ER_PIN_A1 A7                                                        // DC

#elif defined(ER_CPU_FANSTEL)                                                   // Fanstel module
    #include <Arduino.h>
    #include <SPI_RF.h>
    #define SPI_SPEED_WRITE                 SPI_8M                              // 8 MHz // SPI_8M is defined but 8MHz is out-of-spec for the nRF51822, is it ?
    #define SPI_SPEED_READ                  SPI_4M                              // 4 MHz
    #define SPI_SPEED_SLOW                  SPI_2M                              // must be slower then 2 MHz before PLL is configured
    #define SPI_MODE_LCD                    SPI_MODE2

    #define _spiwrite(c)                    SPI_RF.write(c)
    #define _spiwrite16(d)                  SPI_RF.write16(d)
    #define _spiwritedma(wbuf, length)      SPI_RF.write(wbuf, length)

    #define _spiread(r)                     r = SPI_RF.transfer(0x00);
    #define _spibegin()                     SPI_RF.begin()
    #define _spisetDataMode(datamode)       SPI_RF.setSPIMode(datamode)
    #define _spisetBitOrder(order)          SPI_RF.setBitORDER(order)
    #define _spisetSpeed(s)                 SPI_RF.setFrequency(s)

    #define _spiCSLow                       NRF_GPIO->OUTCLR = (1ul << P0_10)
    #define _spiCSHigh                      NRF_GPIO->OUTSET = (1ul << P0_10)

    #define LED_BUILTIN D13                                                     //P0_15 (Pin_nRF51822_to_Arduino should be used)
    #define ER_PIN_BL D4                                                        //D4 - P0_21
    #define ER_PIN_DC D1                                                        //D1 Shared with TX - P0_9 (cores\RBL_nRF51822\pin_transform.cpp)
    #define ER_PIN_LCD_RESET D7                                                 //D7 - P0_17
    #define ER_PIN_SD_CS A5                                                     //D14 - P0_29
    #define ER_PIN_TP_CS D6                                                     //D6 - P0_16
    #define ER_PIN_TP_IRQ D5                                                    //D5 - P0_23
    #define ER_PIN_LCD_CS D2                                                    //D2 - SDA - P0_10
    #define ER_PIN_MOSI A3                                                      //A3 - D11 - P0_12
    #define ER_PIN_MISO A4                                                      //A4 - D12 - P0_13
    #define ER_PIN_SCK SCK                                                      //D3 - SCL - P0_8

    #define ER_PIN_MISO2
    #define ER_PIN_MOSI2
    #define ER_PIN_SCK2
    #define ER_PIN_CS2
    #define ER_PIN_D6
    #define ER_PIN_D7
    #define ER_PIN_SDA
    #define ER_PIN_SCL
    #define ER_PIN_RX D0
    #define ER_PIN_TX D1

#elif defined(ER_CPU_ARDUINO)                                                   // All other boards presume the Arduino headers are used, 32-bit boards work better, generic SPI commands are used
    #include <Arduino.h>
    #include <SPI.h>

    #define SPI_SPEED_WRITE                 SPI_CLOCK_DIV2                      // 8MHz with a 16MHz base clock
    #define SPI_SPEED_READ                  SPI_CLOCK_DIV4                      // 4MHz
    #define SPI_SPEED_SLOW                  SPI_CLOCK_DIV8                      // 2MHz
    #define SPI_MODE_LCD                    SPI_MODE3

    #define _spiwrite(c)                    SPI.transfer(c)
    #define _spiwrite16(d)                  SPI.transfer16(d)

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 setClockDivider(s)

    #define _spiCSLow                       digitalWrite(ER_PIN_LCD_CS, LOW)
    #define _spiCSHigh                      digitalWrite(ER_PIN_LCD_CS, HIGH)

    #define ER_PIN_BL (8)
    #define ER_PIN_DC (7)
    #define ER_PIN_LCD_RESET (9)
    #define ER_PIN_SD_CS (5)
    #define ER_PIN_TP_CS (6)
    #define ER_PIN_TP_IRQ(4)
    #define ER_PIN_LCD_CS PIN_SPI_SS
    #define ER_PIN_MOSI PIN_SPI_MOSI
    #define ER_PIN_MISO PIN_SPI_MISO
    #define ER_PIN_SCK PIN_SPI_SCK

#else                                                                           // Safeguard, should never occur
    #error "Your CPU board is not supported on your X-Graph LCD module."
#endif

#define RA8875_PWRR                     0x01                                    // PWRR [0x01] Power and Display Control Register
#define RA8875_PWRR_DISPON              0x80                                    // LCD Display off: 0 = off / 1 = on
#define RA8875_PWRR_DISPOFF             0x00
#define RA8875_PWRR_SLEEP               0x02                                    // Sleep mode: 0 = normal / 1 = sleep
#define RA8875_PWRR_NORMAL              0x00
#define RA8875_PWRR_SOFTRESET           0x01
#define RA8875_MRWC                     0x02                                    // MRWC [0x02] Memory Read/Write Command
#define RA8875_CMDWRITE                 0x8000
#define RA8875_CMDREAD                  0xC0
#define RA8875_DATAWRITE                0x00
#define RA8875_DATAREAD                 0x40
#define RA8875_STATREG                  0x40
#define RA8875_PCSR                     0x04                                    // PCSR [0x04] Pixel Clock Setting Register
#define RA8875_PCSR_PDATR               0x00                                    // PCLK Inversion: 0 = PDAT at PLCK rising / 1 = PDAT at PLCK falling
#define RA8875_PCSR_PDATL               0x80
#define RA8875_PCSR_CLK                 0x00                                    // PCLK period = System Clock period
#define RA8875_PCSR_2CLK                0x01                                    //             = 2x System Clock period
#define RA8875_PCSR_4CLK                0x02                                    //             = 4x System Clock period
#define RA8875_PCSR_8CLK                0x03                                    //             = 8x System Clock period
#define RA8875_SROC                     0x05                                    // SROC [0x05] Serial Flash/ROM Configuration (not used)
#define RA8875_SFCLR                    0x06                                    // SFCLR [0x06] Serial Flash/ROM CLK (not used)
#define RA8875_SYSR                     0x10                                    // SYSR [0x10] System Configuration Register
#define RA8875_SYSR_8BPP                0x00                                    // Color Depth = 256 colors
#define RA8875_SYSR_16BPP               0x0C                                    //             = 65k colors
#define RA8875_SYSR_MCU8                0x00                                    // MCU Interface = 8-bit
#define RA8875_SYSR_MCU16               0x03                                    //               = 16-bit
#define RA8875_HDWR                     0x14                                    // HDWR [0x14] LCD Horizontal Display Width Register (bits 6->0 and max = 0x63 -> (0x63+1)*8 = 800)
#define RA8875_HNDFTR                   0x15                                    // HNDFTR [0x15] Horizontal Non-Display Period Fine Tuning Option Register = bits 3->0
#define RA8875_HNDFTR_DE_HIGH           0x00                                    // DE polarity = high
#define RA8875_HNDFTR_DE_LOW            0x80                                    //             = low
#define RA8875_HNDR                     0x16                                    // HNDR [0x016] LCD Horizontal Non-Display Period Register (bits 4->0 -> (HNDR + 1) * 8)
#define RA8875_HSTR                     0x17                                    // HSTR [0x17] HSYNC Start Position Register  (bits 4->0 -> (HSTR + 1) * 8)
#define RA8875_HPWR                     0x18                                    // HPWR [0x18] HSYNC Pulse Width Register (bits 4->0 -> (HPWR + 1) * 8)
#define RA8875_HPWR_LOW                 0x00                                    // HSYNC Polarity 0 = low
#define RA8875_HPWR_HIGH                0x80                                    //                 1 = high
#define RA8875_VDHR0                    0x19                                    // LCD Vertical Display Height Register 0
#define RA8875_VDHR1                    0x1A                                    // LCD Vertical Display Height Register 1
#define RA8875_VNDR0                    0x1B                                    // LCD Vertical Non-Display Period Register 0
#define RA8875_VNDR1                    0x1C                                    // LCD Vertical Non-Display Period Register 1
#define RA8875_VSTR0                    0x1D                                    // VSYNC Start Position Register 0
#define RA8875_VSTR1                    0x1E                                    // VSYNC Start Position Register 1
#define RA8875_VPWR                     0x1F                                    // VSYNC Pulse Width Register
#define RA8875_VPWR_LOW                 0x00
#define RA8875_VPWR_HIGH                0x80
#define RA8875_DPCR                     0x20                                    // DPCR [0x20] Display Configuration Register
#define RA8875_DPCR_ONE_LAYER           0x00                                    // 1 layer
#define RA8875_DPCR_TWO_LAYERS          0x80                                    // 2 layers
#define RA8875_DPCR_HDIR_NORMAL         0x00                                    // Horizontal scan direction SEG0 -> SEG(n-1)
#define RA8875_DPCR_HDIR_REVERSE        0x08                                    // reversed
#define RA8875_DPCR_VDIR_NORMAL         0x00                                    // Vertical scan direction COM0 -> COM(n-1)
#define RA8875_DPCR_VDIR_REVERSE        0x04                                    // reversed
#define RA8875_FNCR0                    0x21                                    // FNCR0 [0x21] Font Control Register 0
#define RA8875_FNCR0_CG_MASK            0xa0
#define RA8875_FNCR0_CGROM              0x00                                    // Font selection in text mode: CGROM
#define RA8875_FNCR0_CGRAM              0x80                                    //                              CGRAM
#define RA8875_FNCR0_INTERNAL_CGROM     0x00                                    // External or Internal CGROM: Internal (RA8875_SFRSET = 0)
#define RA8857_FNCR0_EXTERNAL_CGROM     0x20                                    //                           : External (RA8875_FWTSET bit 6,7 = 0)
#define RA8875_FNCR0_8859_MASK          0x03
#define RA8857_FNCR0_8859_1             0x00                                    // Font Selection for internal CGROM
#define RA8857_FNCR0_8859_2             0x01
#define RA8857_FNCR0_8859_3             0x02
#define RA8857_FNCR0_8859_4             0x03
#define RA8875_FNCR1                    0x22                                    // FNCR1 [0x22] Font Control Register 1
#define RA8875_FNCR1_ALIGNMENT_OFF      0x00                                    // Full Alignment disabled
#define RA8875_FNCR1_ALIGNMENT_ON       0x80                                    //                enabled
#define RA8875_FNCR1_TRANSPARENT_OFF    0x00                                    // Font Transparency disabled
#define RA8875_FNCR1_TRANSPARENT_ON     0x40                                    //                   enabled
#define RA8875_FNCR1_NORMAL             0x00                                    // Font Rotation disabled
#define RA8875_FNCR1_90DEGREES          0x10                                    //               90 degrees
#define RA8875_FNCR1_SCALE_MASK         0x0f
#define RA8875_FNCR1_SCALE_HOR_1        0x00                                    // Font Horizontal Enlargment disabled
#define RA8875_FNCR1_SCALE_HOR_2        0x04                                    //                            x2
#define RA8875_FNCR1_SCALE_HOR_3        0x08                                    //                            x3
#define RA8875_FNCR1_SCALE_HOR_4        0x0c                                    //                            x4
#define RA8875_FNCR1_SCALE_VER_1        0x00                                    // Font Vertical Enlargment disabled
#define RA8875_FNCR1_SCALE_VER_2        0x01                                    //                          x2
#define RA8875_FNCR1_SCALE_VER_3        0x02                                    //                          x3
#define RA8875_FNCR1_SCALE_VER_4        0x03                                    //                          x4
#define RA8875_CGSR                     0x23                                    // CGSR [0x23] CGRAM Select Register
#define RA8875_HOFS0                    0x24                                    // HOFS0 [0x24] Horizontal Scroll Offset Register 0
#define RA8875_HOFS1                    0x25                                    // HOFS1 [0x25] Horizontal Scroll Offset Register 1 (bits 2->0)
#define RA8875_VOFS0                    0x26                                    // VOFS0 [0x26] Vertical Scroll Offset Register 0
#define RA8875_VOFS1                    0x27                                    // VOFS1 [0x27] Vertical Scroll Offset Register 1 (bits 1->0)
#define RA8875_FLDR                     0x29                                    // FLDR [0x29] Font Line Distance Setting Register (bits 4->0)
#define RA8875_F_CURXL                  0x2A                                    // CURXL [0x2A] Font Write Cursor Horizontal Position Register 0
#define RA8875_F_CURXH                  0x2B                                    // CURXH [0x2B] Font Write Cursor Horizontal Position Register 1 (bits 1->0)
#define RA8875_F_CURYL                  0x2C                                    // CURYL [0x2C] Font Write Cursor Vertical Position Register 0
#define RA8875_F_CURYH                  0x2D                                    // CURYH [0x2D] Font Write Cursor Vertical Position Register 1 (bit 0)
#define RA8875_FWTSET                   0x2E                                    // FWTSET [0x2E] Font Write Type Setting Register
#define RA8875_FWTSET_SIZE_MASK         0xc0
#define RA8875_FWTSET_16X16             0x00                                    // Font set: 16x16 (full) - 8x16 (half) - nx16 (var)
#define RA8875_FWTSET_24X24             0x40                                    // Font set: 24x24 (full) - 12x24 (half) - nx24 (var)
#define RA8875_FWTSET_32X32             0x80                                    // Font set: 32x32 (full) - 16x32 (half) - nx32 (var)
#define RA8875_FWTSET_WIDTH_MASK        0x3F                                    // Font to Font Width Setting (0 to 63 pixels)
#define RA8875_SFRSET                   0x2F                                    // SFRSET [0x0F] Serial Font ROM Setting
#define RA8875_HSAW0                    0x30                                    // HSAW0 [0x30] Horizontal Start Point 0 of Active Window
#define RA8875_HSAW1                    0x31                                    // HSAW1 [0x31] Horizontal Start Point 1 of Active Window
#define RA8875_VSAW0                    0x32                                    // VSAW0 [0x32] Vertical   Start Point 0 of Active Window
#define RA8875_VSAW1                    0x33                                    // VSAW1 [0x32] Vertical   Start Point 1 of Active Window
#define RA8875_HEAW0                    0x34                                    // HEAW0 [0x34] Horizontal End   Point 0 of Active Window
#define RA8875_HEAW1                    0x35                                    // HEAW1 [0x35] Horizontal End   Point 1 of Active Window
#define RA8875_VEAW0                    0x36                                    // VEAW0 [0x36] Vertical   End   Point of Active Window 0
#define RA8875_VEAW1                    0x37                                    // VEAW1 [0x37] Vertical   End   Point of Active Window 1
#define RA8875_HSSW0                    0x38                                    // HSSW0 [0x38] Horizontal Start Point 0 of Scroll Window
#define RA8875_HSSW1                    0x39                                    // HSSW1 [0x39] Horizontal Start Point 1 of Scroll Window
#define RA8875_VSSW0                    0x3A                                    // VSSW0 [0x3A] Vertical      Start Point 0 of Scroll Window
#define RA8875_VSSW1                    0x3B                                    // VSSW1 [0x3B] Vertical      Start Point 1 of Scroll Window
#define RA8875_HESW0                    0x3C                                    // HESW0 [0x3C] Horizontal End   Point 0 of Scroll Window
#define RA8875_HESW1                    0x3D                                    // HESW1 [0x3D] Horizontal End   Point 1 of Scroll Window
#define RA8875_VESW0                    0x3E                                    // VESW0 [0x3E] Vertical      End   Point 0 of Scroll Window
#define RA8875_VESW1                    0x3F                                    // VESW1 [0x3F] Vertical      End   Point 1 of Scroll Window
#define RA8875_MWCR0                    0x40                                    //Memory Write Control Register 0
#define RA8875_MWCR0_GFXMODE            0x00
#define RA8875_MWCR0_TXTMODE            0x80
#define RA8875_MWCR0_NO_CURSOR          0x00
#define RA8875_MWCR0_CURSOR             0x40
#define RA8875_MWCR0_CURSOR_NORMAL      0x00
#define RA8875_MWCR0_CURSOR_BLINK       0x20
#define RA8875_MWCR0_MEMWRDIR_MASK      0x0c
#define RA8875_MWCR0_MEMWRDIR_LT        0x00
#define RA8875_MWCR0_MEMWRDIR_RT        0x04
#define RA8875_MWCR0_MEMWRDIR_TL        0x08
#define RA8875_MWCR0_MEMWRDIR_DL        0x0c
#define RA8875_MWCR0_MEMWR_CUR_INC      0x00
#define RA8875_MWCR0_MEMWR_NO_INC       0x02
#define RA8875_MWCR0_MEMRD_CUR_INC      0x00
#define RA8875_MWCR0_MEMRD_NO_INC       0x01
#define RA8875_MWCR1                    0x41
#define RA8875_BTCR                     0x44
#define RA8875_MRCD                     0x45                                    //Memory Read Cursor Direction
#define RA8875_CURH0                    0x46                                    //Memory Write Cursor Horizontal Position Register 0
#define RA8875_CURH1                    0x47                                    //Memory Write Cursor Horizontal Position Register 1
#define RA8875_CURV0                    0x48                                    //Memory Write Cursor Vertical Position Register 0
#define RA8875_CURV1                    0x49                                    //Memory Write Cursor Vertical Position Register 1
#define RA8875_RCURH0                   0x4A                                    //Memory Read Cursor Horizontal Position Register 0
#define RA8875_RCURH1                   0x4B                                    //Memory Read Cursor Horizontal Position Register 1
#define RA8875_RCURV0                   0x4C                                    //Memory Read Cursor Vertical Position Register 0
#define RA8875_RCURV1                   0x4D
#define RA8875_CURHS                    0x4E
#define RA8875_CURVS                    0x4F
#define RA8875_BECR0                    0x50                                    //BTE Function Control Register 0
#define RA8875_BECR1                    0x51
#define RA8875_LTPR0                    0x52
#define RA8875_LTPR1                    0x53                                    //Layer Transparency Register 1
#define RA8875_HSBE0                    0x54                                    //Horizontal Source Point 0 of BTE
#define RA8875_HSBE1                    0x55                                    //Horizontal Source Point 1 of BTE
#define RA8875_VSBE0                    0x56                                    //Vertical Source Point 0 of BTE
#define RA8875_VSBE1                    0x57                                    //Vertical Source Point 1 of BTE
#define RA8875_HDBE0                    0x58                                    //Horizontal Destination Point 0 of BTE
#define RA8875_HDBE1                    0x59                                    //Horizontal Destination Point 1 of BTE
#define RA8875_VDBE0                    0x5A                                    //Vertical Destination Point 0 of BTE
#define RA8875_VDBE1                    0x5B                                    //Vertical Destination Point 1 of BTE
#define RA8875_BEWR0                    0x5C                                    //BTE Width Register 0
#define RA8875_BEWR1                    0x5D                                    //BTE Width Register 1
#define RA8875_BEHR0                    0x5E                                    //BTE Height Register 0
#define RA8875_BEHR1                    0x5F
#define RA8875_PTNO                     0x66
#define RA8875_BTEROP_SOURCE            0xC0                                    //Overwrite dest with source (no mixing) *****THIS IS THE DEFAULT OPTION****
#define RA8875_BTEROP_BLACK             0x00                                    //all black
#define RA8875_BTEROP_WHITE             0xf0                                    //all white
#define RA8875_BTEROP_DEST              0xA0                                    //destination unchanged
#define RA8875_BTEROP_ADD               0xE0                                    //ADD (brighter)
#define RA8875_BTEROP_SUBTRACT          0x20
#define RA8875_BGCR0                    0x60                                    //Background Color Register 0 (R)
#define RA8875_BGCR1                    0x61                                    //Background Color Register 1 (G)
#define RA8875_BGCR2                    0x62                                    //Background Color Register 2 (B)
#define RA8875_FGCR0                    0x63                                    //Foreground Color Register 0 (R)
#define RA8875_FGCR1                    0x64                                    //Foreground Color Register 1 (G)
#define RA8875_FGCR2                    0x65                                    //Foreground Color Register 2 (B)
#define RA8875_BGTR0                    0x67                                    //Background Color Register for Transparent 0 (R)
#define RA8875_BGTR1                    0x68                                    //Background Color Register for Transparent 1 (G)
#define RA8875_BGTR2                    0x69
#define RA8875_TPCR0                    0x70                                    //Touch Panel Control Register 0
#define RA8875_TPCR0_ENABLE             0x80
#define RA8875_TPCR0_DISABLE            0x00
#define RA8875_TPCR0_WAIT_512CLK        0x00
#define RA8875_TPCR0_WAIT_1024CLK       0x10
#define RA8875_TPCR0_WAIT_2048CLK       0x20
#define RA8875_TPCR0_WAIT_4096CLK       0x30
#define RA8875_TPCR0_WAIT_8192CLK       0x40
#define RA8875_TPCR0_WAIT_16384CLK      0x50
#define RA8875_TPCR0_WAIT_32768CLK      0x60
#define RA8875_TPCR0_WAIT_65536CLK      0x70
#define RA8875_TPCR0_WAKEENABLE         0x08
#define RA8875_TPCR0_WAKEDISABLE        0x00
#define RA8875_TPCR0_ADCCLK_DIV1        0x00
#define RA8875_TPCR0_ADCCLK_DIV2        0x01
#define RA8875_TPCR0_ADCCLK_DIV4        0x02
#define RA8875_TPCR0_ADCCLK_DIV8        0x03
#define RA8875_TPCR0_ADCCLK_DIV16       0x04
#define RA8875_TPCR0_ADCCLK_DIV32       0x05
#define RA8875_TPCR0_ADCCLK_DIV64       0x06
#define RA8875_TPCR0_ADCCLK_DIV128      0x07
#define RA8875_TPCR1                    0x71                                    //Touch Panel Control Register 1
#define RA8875_TPCR1_AUTO               0x00
#define RA8875_TPCR1_MANUAL             0x40
#define RA8875_TPCR1_VREFINT            0x00
#define RA8875_TPCR1_VREFEXT            0x20
#define RA8875_TPCR1_DEBOUNCE           0x04
#define RA8875_TPCR1_NODEBOUNCE         0x00
#define RA8875_TPCR1_IDLE               0x00
#define RA8875_TPCR1_WAIT               0x01
#define RA8875_TPCR1_LATCHX             0x02
#define RA8875_TPCR1_LATCHY             0x03
#define RA8875_TPXH                     0x72                                    //Touch Panel X High Byte Data Register
#define RA8875_TPYH                     0x73                                    //Touch Panel Y High Byte Data Register
#define RA8875_TPXYL                    0x74
#define RA8875_GCHP0                    0x80                                    //Graphic Cursor Horizontal Position Register 0
#define RA8875_GCHP1                    0x81                                    //Graphic Cursor Horizontal Position Register 1
#define RA8875_GCVP0                    0x82                                    //Graphic Cursor Vertical Position Register 0
#define RA8875_GCVP1                    0x83                                    //Graphic Cursor Vertical Position Register 0
#define RA8875_GCC0                     0x84                                    //Graphic Cursor Color 0
#define RA8875_GCC1                     0x85
#define RA8875_PLLC1                    0x88                                    //PLL Control Register 1
#define RA8875_PLLC1_PLLDIV2            0x80
#define RA8875_PLLC1_PLLDIV1            0x00
#define RA8875_PLLC2                    0x89                                    //PLL Control Register 2
#define RA8875_PLLC2_DIV1               0x00
#define RA8875_PLLC2_DIV2               0x01
#define RA8875_PLLC2_DIV4               0x02
#define RA8875_PLLC2_DIV8               0x03
#define RA8875_PLLC2_DIV16              0x04
#define RA8875_PLLC2_DIV32              0x05
#define RA8875_PLLC2_DIV64              0x06
#define RA8875_PLLC2_DIV128             0x07
#define RA8875_P1CR                     0x8A                                    //PWM1 Control Register
#define RA8875_P1CR_ENABLE              0x80
#define RA8875_P1CR_DISABLE             0x00
#define RA8875_P1CR_CLKOUT              0x10
#define RA8875_P1CR_PWMOUT              0x00
#define RA8875_P1DCR                    0x8B                                    //PWM1 Duty Cycle Register
#define RA8875_P2CR                     0x8C                                    //PWM2 Control Register
#define RA8875_P2CR_ENABLE              0x80
#define RA8875_P2CR_DISABLE             0x00
#define RA8875_P2CR_CLKOUT              0x10
#define RA8875_P2CR_PWMOUT              0x00
#define RA8875_P2DCR                    0x8D                                    //PWM2 Control Register
#define RA8875_PxCR_ENABLE              0x80
#define RA8875_PxCR_DISABLE             0x00
#define RA8875_PxCR_CLKOUT              0x10
#define RA8875_PxCR_PWMOUT              0x00
#define RA8875_PWM_CLK_DIV1             0x00
#define RA8875_PWM_CLK_DIV2             0x01
#define RA8875_PWM_CLK_DIV4             0x02
#define RA8875_PWM_CLK_DIV8             0x03
#define RA8875_PWM_CLK_DIV16            0x04
#define RA8875_PWM_CLK_DIV32            0x05
#define RA8875_PWM_CLK_DIV64            0x06
#define RA8875_PWM_CLK_DIV128           0x07
#define RA8875_PWM_CLK_DIV256           0x08
#define RA8875_PWM_CLK_DIV512           0x09
#define RA8875_PWM_CLK_DIV1024          0x0A
#define RA8875_PWM_CLK_DIV2048          0x0B
#define RA8875_PWM_CLK_DIV4096          0x0C
#define RA8875_PWM_CLK_DIV8192          0x0D
#define RA8875_PWM_CLK_DIV16384         0x0E
#define RA8875_PWM_CLK_DIV32768         0x0F
#define RA8875_MCLR                     0x8E                                    //Memory Clear Control Register
#define RA8875_MCLR_START               0x80
#define RA8875_MCLR_STOP                0x00
#define RA8875_MCLR_READSTATUS          0x80
#define RA8875_MCLR_FULL                0x00
#define RA8875_MCLR_ACTIVE              0x40
#define RA8875_DCR                      0x90                                    //Draw Line/Circle/Square Control Register
#define RA8875_DCR_LINESQUTRI_START     0x80
#define RA8875_DCR_LINESQUTRI_STOP      0x00
#define RA8875_DCR_LINESQUTRI_STATUS    0x80
#define RA8875_DCR_CIRCLE_START         0x40
#define RA8875_DCR_CIRCLE_STATUS        0x40
#define RA8875_DCR_CIRCLE_STOP          0x00
#define RA8875_DCR_FILL                 0x20
#define RA8875_DCR_NOFILL               0x00
#define RA8875_DCR_DRAWLINE             0x00
#define RA8875_DCR_DRAWTRIANGLE         0x01
#define RA8875_DCR_DRAWSQUARE           0x10
#define RA8875_DLHSR0                   0x91                                    //Draw Line/Square Horizontal Start Address Register0
#define RA8875_DLHSR1                   0x92                                    //Draw Line/Square Horizontal Start Address Register1
#define RA8875_DLVSR0                   0x93                                    //Draw Line/Square Vertical Start Address Register0
#define RA8875_DLVSR1                   0x94                                    //Draw Line/Square Vertical Start Address Register1
#define RA8875_DLHER0                   0x95                                    //Draw Line/Square Horizontal End Address Register0
#define RA8875_DLHER1                   0x96                                    //Draw Line/Square Horizontal End Address Register1
#define RA8875_DLVER0                   0x97                                    //Draw Line/Square Vertical End Address Register0
#define RA8875_DLVER1                   0x98                                    //Draw Line/Square Vertical End Address Register0
#define RA8875_DCHR0                    0x99                                    //Draw Circle Center Horizontal Address Register0
#define RA8875_DCHR1                    0x9A                                    //Draw Circle Center Horizontal Address Register1
#define RA8875_DCVR0                    0x9B                                    //Draw Circle Center Vertical Address Register0
#define RA8875_DCVR1                    0x9C                                    //Draw Circle Center Vertical Address Register1
#define RA8875_DCRR                     0x9D                                    //Draw Circle Radius Register
#define RA8875_ELLIPSE                  0xA0                                    //Draw Ellipse/Ellipse Curve/Circle Square Control Register
#define RA8875_ELLIPSE_STATUS           0x80
#define RA8875_ELL_A0                   0xA1                                    //Draw Ellipse/Circle Square Long axis Setting Register0
#define RA8875_ELL_A1                   0xA2                                    //Draw Ellipse/Circle Square Long axis Setting Register1
#define RA8875_ELL_B0                   0xA3                                    //Draw Ellipse/Circle Square Short axis Setting Register0
#define RA8875_ELL_B1                   0xA4                                    //Draw Ellipse/Circle Square Short axis Setting Register1
#define RA8875_DEHR0                    0xA5                                    //Draw Ellipse/Circle Square Center Horizontal Address Register0
#define RA8875_DEHR1                    0xA6                                    //Draw Ellipse/Circle Square Center Horizontal Address Register1
#define RA8875_DEVR0                    0xA7                                    //Draw Ellipse/Circle Square Center Vertical Address Register0
#define RA8875_DEVR1                    0xA8                                    //Draw Ellipse/Circle Square Center Vertical Address Register1
#define RA8875_DTPH0                    0xA9                                    //Draw Triangle Point 2 Horizontal Address Register0
#define RA8875_DTPH1                    0xAA                                    //Draw Triangle Point 2 Horizontal Address Register1
#define RA8875_DTPV0                    0xAB                                    //Draw Triangle Point 2 Vertical Address Register0
#define RA8875_DTPV1                    0xAC
#define RA8875_SSAR0                    0xB0                                    //Source Starting Address REG 0
#define RA8875_SSAR1                    0xB1                                    //Source Starting Address REG 1
#define RA8875_SSAR2                    0xB2
#define RA8875_DTNR0                    0xB4                                    //Block Width REG 0(BWR0) / DMA Transfer Number REG 0
#define RA8875_BWR1                     0xB5                                    //Block Width REG 1
#define RA8875_DTNR1                    0xB6                                    //Block Height REG 0(BHR0) /DMA Transfer Number REG 1
#define RA8875_BHR1                     0xB7                                    //Block Height REG 1
#define RA8875_DTNR2                    0xB8                                    //Source Picture Width REG 0(SPWR0) / DMA Transfer Number REG 2
#define RA8875_SPWR1                    0xB9                                    //Source Picture Width REG 1
#define RA8875_DMACR                    0xBF
#define RA8875_GPI                      0x12
#define RA8875_GPO                      0x13
#define RA8875_GPIOX                    0xC7
#define RA8875_KSCR1                    0xC0                                    //Key-Scan Control Register 1 (KSCR1)
#define RA8875_KSCR2                    0xC1                                    //Key-Scan Controller Register 2 (KSCR2)
#define RA8875_KSDR0                    0xC2                                    //Key-Scan Data Register (KSDR0)
#define RA8875_KSDR1                    0xC3                                    //Key-Scan Data Register (KSDR1)
#define RA8875_KSDR2                    0xC4
#define RA8875_INTC1                    0xF0                                    //Interrupt Control Register1
#define RA8875_INTC2                    0xF1                                    //Interrupt Control Register2
#define RA8875_INTCx_KEY                0x10
#define RA8875_INTCx_DMA                0x08
#define RA8875_INTCx_TP                 0x04
#define RA8875_INTCx_BTE                0x02

#define RA8875_ENABLE_INT_TP            ((uint8_t)(1<<2))
#define RA8875_DISABLE_INT_TP           ((uint8_t)(0<<2))
static const uint8_t _RA8875colorMask[6] = {11,5,0,13,8,3};                     //for color masking, first 3 byte for 65K
const uint16_t COLOR_BLACK              = 0x0000;
const uint16_t COLOR_WHITE              = 0xFFFF;

#define RA8875_BLACK                    0x0000
#define RA8875_BLUE                     0x001F
#define RA8875_RED                      0xF800
#define RA8875_GREEN                    0x07E0
#define RA8875_CYAN                     0x07FF
#define RA8875_MAGENTA                  0xF81F
#define RA8875_YELLOW                   0xFFE0
#define RA8875_WHITE                    0xFFFF

#ifndef bitRead
    #define bitRead(a,b) ((a) & (1<<(b)))
#endif
#ifndef bitWrite
    #define __bitSet(value, bit) ((value) |= (1UL << (bit)))
    #define __bitClear(value, bit) ((value) &= ~(1UL << (bit)))
    #define bitWrite(value, bit, bitvalue) (bitvalue ? __bitSet(value, bit) : __bitClear(value, bit))
#endif
#ifndef PI
    #define PI 3.14159265358979323846
#endif

#if !defined(swapvals)
    #define swapvals(a, b) { typeof(a) t = a; a = b; b = t; }
#endif

#if defined(ER_LCD_USE_GSL1680)
    #include "touch/GSL1680.h"
    #define CTP_WAKE    14
    #define CTP_INTRPT  32
    GSL1680 TS = GSL1680();
#endif

enum RA8875tcursor          { NOCURSOR=0, IBEAM, UNDER, BLOCK };
enum RA8875fontCoding       { ISO_IEC_8859_1, ISO_IEC_8859_2, ISO_IEC_8859_3, ISO_IEC_8859_4 };
enum RA8875boolean          { LAYER1, LAYER2, TRANSPARENT, LIGHTEN, OR, AND, FLOATING };
enum RA8875writes           { L1=0, L2, CGRAM, PATTERN, CURSOR };
enum RA8875scrollMode       { SIMULTANEOUS, LAYER1ONLY, LAYER2ONLY, BUFFERED };
enum RA8875pattern          { P8X8, P16X16 };
enum RA8875btedatam         { CONT, RECT };
enum RA8875btelayer         { SOURCE, DEST };
enum RA8875intlist          { BTE=1,TOUCH=2, DMA=3, KEY=4 };

class ERDisplay  {
  public:
    ERDisplay(void);
    void        begin(void);
    void        displayOn(boolean on);
    void        sleep(boolean sleep);
    void        GPIOX(boolean on);
    void        setSPI();

#if defined(ER_CPU_ESP32)
    spi_dev_t * dev = (volatile spi_dev_t *)(DR_REG_SPI3_BASE);                 // VSPI = SPI3 = default SPI port
    lldesc_t dmadesc[32];                                                       // max dma size = 32 * 4096 words = 128kByte
#endif
    
    void        setActiveWindow(void);
    void        setActiveWindow(int16_t XL,int16_t XR,int16_t YT,int16_t YB);
    void        clearMemory(bool stop=false);
    uint16_t    width(bool absolute=false) const;
    uint16_t    height(bool absolute=false) const;
    void        setRotation(uint8_t rotation);

    void        setX(int16_t x);
    void        setY(int16_t y) ;
    
    void        drawBitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t * image);
    
    void        setColorBpp(uint8_t colors);

    void        PWMout(uint8_t pw,uint8_t p);
    void        brightness(uint8_t val);
    void        backlight(boolean on);
    void        PWMsetup(uint8_t pw,boolean on, uint8_t clock);
    
    uint8_t     readStatus(void);
    void        writeCommand(const uint8_t d);

    void        readTouch(void);

protected:
    int16_t     LCD_WIDTH, LCD_HEIGHT;                                          //absolute
    int16_t     _width, _height;
    
private:
    uint8_t     _lcdtype;
    uint8_t     _pixclk;
    uint8_t     _hsync_nondisp;
    uint8_t     _hsync_start;
    uint8_t     _hsync_pw;
    uint8_t     _hsync_finetune;
    uint8_t     _vsync_nondisp;
    uint8_t     _vsync_start;
    uint8_t     _vsync_pw;
    uint8_t     _pll_div;
    
    bool        _sleep;
    bool        _portrait;
    uint8_t     _rotation;
    int16_t     _activeWindowXL,_activeWindowXR,_activeWindowYT,_activeWindowYB;
    uint8_t     _colorIndex;
    uint8_t     _color_bpp;                                                     //8=256, 16=64K colors
    uint8_t     _brightness;
    uint8_t     _DPCR_Reg;                                                      // Display Configuration [0x20]
    uint8_t     _FNCR0_Reg;                                                     // Font Control Register 0 [0x21]
    uint8_t     _FNCR1_Reg;                                                     // Font Control Register1 [0x22]
    uint8_t     _FWTSET_Reg;                                                    // Font Write Type Setting Register [0x2E]
    uint8_t     _SFRSET_Reg;                                                    // Serial Font ROM Setting [0x2F]
    uint8_t     _INTC1_Reg;                                                     // Interrupt Control Register1 [0xF0]
    volatile uint8_t _MWCR0_Reg;
    
    void        _setSysClock(uint8_t pll1,uint8_t pll2,uint8_t pixclk);
    void        _updateActiveWindow(bool full);
    void        _updateActiveWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
    void        _scanDirection(boolean invertH,boolean invertV);
    void        _line_addressing(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
    void        _writeRegister(const uint8_t reg, uint8_t val);
    uint8_t     _readRegister(const uint8_t reg);
    void        _writeData(uint8_t data);
    void        _writeData16(uint16_t data);
    uint8_t     _readData(bool stat=false);
    boolean     _waitPoll(uint8_t r, uint8_t f);
    void        _waitBusy(uint8_t res=0x80);                                    //0x80, 0x40(BTE busy), 0x01(DMA busy)
};

#endif
