#include "ERDisplay.h"

ERDisplay::ERDisplay(void) {}

void ERDisplay::begin(void) {
	
    _rotation = 0;
	_sleep = false;
    _portrait = false;
    _brightness = 255;
    _activeWindowXL = 0;
    _activeWindowYT = 0;
    _color_bpp = 0;
    _lcdtype = 0;

    pinMode(ER_PIN_LCD_CS, OUTPUT);
    digitalWrite(ER_PIN_LCD_CS, HIGH);
    _spibegin();
    setSPI();
    _spisetSpeed(SPI_SPEED_SLOW);
    
    pinMode(ER_PIN_LCD_RESET, OUTPUT);
	digitalWrite(ER_PIN_LCD_RESET, HIGH);
	delay(10);
	digitalWrite(ER_PIN_LCD_RESET, LOW);
	delay(220);
	digitalWrite(ER_PIN_LCD_RESET, HIGH);
	delay(300);

    pinMode(ER_PIN_SD_CS, OUTPUT);
    digitalWrite(ER_PIN_SD_CS, HIGH);
        
    _DPCR_Reg = RA8875_DPCR_ONE_LAYER + RA8875_DPCR_HDIR_NORMAL + RA8875_DPCR_VDIR_NORMAL;
    _MWCR0_Reg = RA8875_MWCR0_GFXMODE + RA8875_MWCR0_NO_CURSOR + RA8875_MWCR0_CURSOR_NORMAL + RA8875_MWCR0_MEMWRDIR_LT + RA8875_MWCR0_MEMWR_CUR_INC + RA8875_MWCR0_MEMRD_NO_INC;
    _FNCR0_Reg = RA8875_FNCR0_CGROM + RA8875_FNCR0_INTERNAL_CGROM + RA8857_FNCR0_8859_1;
    _FNCR1_Reg = RA8875_FNCR1_ALIGNMENT_OFF + RA8875_FNCR1_TRANSPARENT_OFF + RA8875_FNCR1_NORMAL + RA8875_FNCR1_SCALE_HOR_1 + RA8875_FNCR1_SCALE_VER_1;
    _FWTSET_Reg = RA8875_FWTSET_16X16;
    _SFRSET_Reg = 0b00000000;
    _INTC1_Reg = 0b00000000;

    writeCommand(RA8875_PWRR);
    _writeData(RA8875_PWRR_SOFTRESET);
    delay(20);
    _writeData(RA8875_PWRR_NORMAL);
    delay(200);
    _setSysClock(0x07, RA8875_PLLC2_DIV8, RA8875_PCSR_PDATL | RA8875_PCSR_2CLK);

    setColorBpp(16);

    #ifdef ER_LCD_AUTO
        _writeRegister(RA8875_KSCR1, 0x00);                                     // Disable keyboard scanning
        uint8_t i;
        writeCommand(RA8875_GPI);
        _spiCSLow;
#ifdef _spixread
        _spixread(RA8875_DATAREAD, i);
#else
        _spiwrite(RA8875_DATAREAD);
        delayMicroseconds(50);                                                  // Stabilize time, else first bit is read wrong
        _spiread(i);
#endif
        _spiCSHigh;

        if (i == 0x00) _lcdtype = 0;                                            // Default is lcdtype 0 = 5.0" 800x480
        if (i == 0x01) _lcdtype = 2;                                            // lcdtype 2 = 9.0" 800x480
    #else
        #if defined(ER_LCD_56)
            _lcdtype = 1;
        #elif defined(ER_LCD_90)
            _lcdtype = 2;
        #endif
    #endif

    switch (_lcdtype) {
        case 0:
        case 2:                                                                 // 800x480
            _activeWindowXR  = LCD_WIDTH = _width = 800;
            _activeWindowYB  = LCD_HEIGHT = _height = 480;
            _pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
            _hsync_finetune  = 0;
            _hsync_nondisp   = 26;
            _hsync_start     = 32;
            _hsync_pw        = 96;
            _vsync_nondisp   = 32;
            _vsync_start     = 23;
            _vsync_pw        = 2;
            _pll_div         = RA8875_PLLC2_DIV4;
            break;
        case 1:                                                                 // 640x480
            _activeWindowXR  = LCD_WIDTH = _width = 640;
            _activeWindowYB  = LCD_HEIGHT = _height = 480;
            _pixclk          = RA8875_PCSR_2CLK;
            _hsync_finetune  = 5;
            _hsync_nondisp   = 127;
            _hsync_start     = 16;
            _hsync_pw        = 8;
            _vsync_nondisp   = 11;
            _vsync_start     = 15;
            _vsync_pw        = 2;
            _pll_div         = RA8875_PLLC2_DIV4;
            break;
    }
    
    _writeRegister(RA8875_HDWR, ((LCD_WIDTH)/8) - 1);		                    // LCD Horizontal Display Width Register
    _writeRegister(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + _hsync_finetune);     // Horizontal Non-Display Period Fine Tuning Option Register
    _writeRegister(RA8875_HNDR, (_hsync_nondisp-_hsync_finetune - 2) / 8);		// LCD Horizontal Non-Display Period Register
    _writeRegister(RA8875_HSTR, _hsync_start/8 - 1);		                    // HSYNC Start Position Register
    _writeRegister(RA8875_HPWR, RA8875_HPWR_LOW + (_hsync_pw/8 - 1));		    // HSYNC Pulse Width Register
    _writeRegister(RA8875_VDHR0, (uint16_t)(((LCD_HEIGHT) - 1) & 0xFF));         // LCD Vertical Display Height Register0
    _writeRegister(RA8875_VDHR0+1, (uint16_t)((LCD_HEIGHT) - 1) >> 8);	        // LCD Vertical Display Height Register1
    _writeRegister(RA8875_VNDR0, _vsync_nondisp-1);                             // LCD Vertical Non-Display Period Register 0
    _writeRegister(RA8875_VNDR0+1, _vsync_nondisp >> 8);	                    // LCD Vertical Non-Display Period Register 1
    _writeRegister(RA8875_VSTR0, _vsync_start-1);                               // VSYNC Start Position Register 0
    _writeRegister(RA8875_VSTR0+1, _vsync_start >> 8);	                        // VSYNC Start Position Register 1
    _writeRegister(RA8875_VPWR, RA8875_VPWR_LOW + _vsync_pw - 1);               // VSYNC Pulse Width Register
    
    _setSysClock(0x0b, _pll_div, _pixclk);
    _updateActiveWindow(true);									                // set the whole screen as active
    delay(10);

    _spisetSpeed(SPI_SPEED_WRITE);
    delay(1);
    
    clearMemory();
    delay(1);
	displayOn(true);
    delay(1);
    backlight(true);
	setRotation(0);

    #if defined(ER_LCD_USE_GSL1680)
        TS.begin(CTP_WAKE, CTP_INTRPT);
    #endif
}

void ERDisplay::_setSysClock(uint8_t pll1,uint8_t pll2,uint8_t pixclk)
{
    _writeRegister(RA8875_PLLC1,pll1);
    delay(1);
    _writeRegister(RA8875_PLLC1+1,pll2);
    delay(1);
    _writeRegister(RA8875_PCSR,pixclk);
    delay(1);
}

void ERDisplay::displayOn(boolean on)
{
    on == true ? _writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON) : _writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

void ERDisplay::setSPI(void) {
    _spisetDataMode(SPI_MODE_LCD);
    _spisetBitOrder(MSBFIRST);
    _spisetSpeed(SPI_SPEED_WRITE);
#ifdef _spisetBitLen
    _spisetBitLen;
#endif
}

boolean ERDisplay::_waitPoll(uint8_t regname, uint8_t waitflag)
{
	uint8_t temp;
	unsigned long timeout = millis();
	
	while (1) {
		temp = _readRegister(regname);
		if (!(temp & waitflag)) return true;
		if ((millis() - timeout) > 20) return false;                            //emergency exit! Should never occur.
	}  
	return false;
}

void ERDisplay::_waitBusy(uint8_t res)
{
	uint8_t temp; 	
	unsigned long start = millis();
    
	do {
		temp = readStatus();
        if ((millis() - start) > 10) return;
    } while ((temp & res) == res);
}

void ERDisplay::sleep(boolean sleep)
{
    if (_sleep != sleep){                                                       // only when needed
        _sleep = sleep;
        if (_sleep == true){
            _spisetSpeed(SPI_SPEED_SLOW);
            _setSysClock(0x07,RA8875_PLLC2_DIV8,RA8875_PCSR_PDATR | RA8875_PCSR_4CLK);
            _writeRegister(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
            delay(100);
        } else {
            _writeRegister(RA8875_PWRR, RA8875_PWRR_DISPOFF);
            delay(100);
            _setSysClock(0x07, RA8875_PLLC2_DIV8, _pixclk);
            delay(20);
            _writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);//disp on
            delay(20);
            _spisetSpeed(SPI_SPEED_WRITE);
            _setSysClock(0x0b, _pll_div, _pixclk);
        }
    }
}

void ERDisplay::setActiveWindow(void)
{
    _activeWindowXL = 0; _activeWindowXR = LCD_WIDTH;
    _activeWindowYT = 0; _activeWindowYB = LCD_HEIGHT;
    if (_portrait){swapvals(_activeWindowXL,_activeWindowYT); swapvals(_activeWindowXR,_activeWindowYB);}
    _updateActiveWindow(true);
}

void ERDisplay::setActiveWindow(int16_t XL,int16_t XR ,int16_t YT ,int16_t YB)
{
    if (_portrait) {swapvals(XL,YT); swapvals(XR,YB);}
    
    if (XR >= LCD_WIDTH) XR = LCD_WIDTH;
    if (YB >= LCD_HEIGHT) YB = LCD_HEIGHT;
    
    _activeWindowXL = XL; _activeWindowXR = XR;
    _activeWindowYT = YT; _activeWindowYB = YB;
    _updateActiveWindow(false);
}

void ERDisplay::_updateActiveWindow(bool full)
{
    if (full){
        // X
        _writeRegister(RA8875_HSAW0,    0x00);
        _writeRegister(RA8875_HSAW0 + 1,0x00);
        _writeRegister(RA8875_HEAW0,    (LCD_WIDTH) & 0xFF);
        _writeRegister(RA8875_HEAW0 + 1,(LCD_WIDTH) >> 8);
        // Y
        _writeRegister(RA8875_VSAW0,    0x00);
        _writeRegister(RA8875_VSAW0 + 1,0x00);
        _writeRegister(RA8875_VEAW0,    (LCD_HEIGHT) & 0xFF);
        _writeRegister(RA8875_VEAW0 + 1,(LCD_HEIGHT) >> 8);
    } else {
        // X
        _writeRegister(RA8875_HSAW0,    _activeWindowXL & 0xFF);
        _writeRegister(RA8875_HSAW0 + 1,_activeWindowXL >> 8);
        _writeRegister(RA8875_HEAW0,    _activeWindowXR & 0xFF);
        _writeRegister(RA8875_HEAW0 + 1,_activeWindowXR >> 8);
        // Y
        _writeRegister(RA8875_VSAW0,     _activeWindowYT & 0xFF);
        _writeRegister(RA8875_VSAW0 + 1,_activeWindowYT >> 8);
        _writeRegister(RA8875_VEAW0,    _activeWindowYB & 0xFF);
        _writeRegister(RA8875_VEAW0 + 1,_activeWindowYB >> 8);
    }
}

void ERDisplay::clearMemory(bool stop)
{
    uint8_t temp;
    temp = _readRegister(RA8875_MCLR);
    stop == true ? temp &= ~RA8875_MCLR_START : temp |= RA8875_MCLR_START;
    _writeData(temp);
    if (!stop) _waitBusy(0x80);
}

uint16_t ERDisplay::width(bool absolute) const
{ 
	if (absolute) return LCD_WIDTH;
	return _width; 
}

uint16_t ERDisplay::height(bool absolute) const
{ 
	if (absolute) return LCD_HEIGHT;
	return _height; 
}

void ERDisplay::_scanDirection(boolean invertH,boolean invertV)
{
	invertH == true ? _DPCR_Reg |= RA8875_DPCR_HDIR_REVERSE : _DPCR_Reg &= ~RA8875_DPCR_HDIR_REVERSE;
	invertV == true ? _DPCR_Reg |= RA8875_DPCR_VDIR_REVERSE : _DPCR_Reg &= ~RA8875_DPCR_VDIR_REVERSE;
	_writeRegister(RA8875_DPCR,_DPCR_Reg);
}

void ERDisplay::setRotation(uint8_t rotation)//0.69b32 - less code
{
	_rotation = rotation % 4;                                                   //limit to the range 0-3
	switch (_rotation) {
	case 0:
		//default
		_portrait = false;
		_scanDirection(0,0);
    break;
	case 1:
		//90
		_portrait = true;
		_scanDirection(1,0);
    break;
	case 2:
		//180
		_portrait = false;
		_scanDirection(1,1);
    break;
	case 3:
		//270
		_portrait = true;
		_scanDirection(0,1);
    break;
	}
    
	if (_portrait){
		_width = LCD_HEIGHT;
		_height = LCD_WIDTH;
	} else {
		_width = LCD_WIDTH;
		_height = LCD_HEIGHT;
	}
    
    if (_portrait) _FNCR1_Reg |= RA8875_FNCR1_90DEGREES; else _FNCR1_Reg &= ~RA8875_FNCR1_90DEGREES;
    _writeRegister(RA8875_FNCR1,_FNCR1_Reg);                                    // Rotates fonts 90 degrees (for build-in fonts that is)
    setActiveWindow();
}

void ERDisplay::setX(int16_t x)
{
    if (x < 0) x = 0;
    if (_portrait){
        if (x >= LCD_HEIGHT) x = LCD_HEIGHT-1;
        _writeRegister(RA8875_CURV0, x & 0xFF);
        _writeRegister(RA8875_CURV0+1, x >> 8);
    } else {
        if (x >= LCD_WIDTH) x = LCD_WIDTH-1;
        _writeRegister(RA8875_CURH0, x & 0xFF);
        _writeRegister(RA8875_CURH0+1, (x >> 8));
    }
}

void ERDisplay::setY(int16_t y)
{
    if (y < 0) y = 0;
    if (_portrait){
        if (y >= LCD_WIDTH) y = LCD_WIDTH-1;
        _writeRegister(RA8875_CURH0, y & 0xFF);
        _writeRegister(RA8875_CURH0+1, (y >> 8));
    } else {
        if (y >= LCD_HEIGHT) y = LCD_HEIGHT-1;
        _writeRegister(RA8875_CURV0, y & 0xFF);
        _writeRegister(RA8875_CURV0+1, y >> 8);
    }
}

void ERDisplay::drawBitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t * image) {
    uint32_t count;

    uint32_t i;
    i = (uint32_t)image;
    
    setActiveWindow(x1,x2,y1,y2);
    setX(x1);
    setY(y1);
    count = (x2 - x1 + 1) * (y2 - y1 + 1);
    writeCommand(RA8875_MRWC);
    _spiCSLow;
#ifdef _spixwritedma
    _spixwritedma(RA8875_DATAWRITE, image, count);
#else
    _spiwrite(RA8875_DATAWRITE);
    _spiwritedma(image, count);
#endif
    _spiCSHigh;
}

void ERDisplay::setColorBpp(uint8_t colors)
{
	if (colors != _color_bpp){                                                  //only if necessary
		if (colors < 16) {
			_color_bpp = 8;
			_colorIndex = 3;
			_writeRegister(RA8875_SYSR, RA8875_SYSR_8BPP);
		} else if (colors > 8) {                                                //65K
			_color_bpp = 16;
			_colorIndex = 0;
			_writeRegister(RA8875_SYSR, RA8875_SYSR_16BPP);
		}
	}
}

void ERDisplay::PWMout(uint8_t pw,uint8_t p)
{
	uint8_t reg;
	pw > 1 ? reg = RA8875_P2DCR : reg = RA8875_P1DCR;
	_writeRegister(reg, p);
}

void ERDisplay::brightness(uint8_t val)
{
	_brightness = val;
	PWMout(1,_brightness);
}

void ERDisplay::backlight(boolean on)
{
	if (on == true){
        PWMsetup(1,true, RA8875_PWM_CLK_DIV1024);                               //setup PWM ch 1 for backlight
        PWMout(1,_brightness);                                                  //turn on PWM1
    } else {
        PWMsetup(1,false, RA8875_PWM_CLK_DIV1024);                              //setup PWM ch 1 for backlight
	}
}

void ERDisplay::PWMsetup(uint8_t pw,boolean on, uint8_t clock)
{
	uint8_t reg;
	uint8_t set;
	if (pw > 1){
		reg = RA8875_P2CR;
		on == true ? set = RA8875_PxCR_ENABLE : set = RA8875_PxCR_DISABLE;
	} else {
		reg = RA8875_P1CR;
		on == true ? set = RA8875_PxCR_ENABLE : set = RA8875_PxCR_DISABLE;
	}
	_writeRegister(reg,(set | (clock & 0xF)));
}

void ERDisplay::_writeRegister(const uint8_t reg, uint8_t val)
{
    _spiCSLow;                                                                  //writeCommand(reg);
    _spiwrite16(RA8875_CMDWRITE+reg);
    _spiCSHigh;
    _spiCSHigh;
    _spiCSLow;
    _spiwrite16(val);                                                           // RA8875_DATAWRITE = 0x00 (so skip the addition to speed up things)
    _spiCSHigh;
}

uint8_t ERDisplay::_readRegister(const uint8_t reg)
{
	writeCommand(reg);
	return _readData(false);
}

void ERDisplay::_writeData(uint8_t data)
{
    _spiCSLow;
    _spiwrite16(data);                                                          // RA8875_DATAWRITE = 0x00 (so skip the addition to speed up things)
    _spiCSHigh;
}

void  ERDisplay::_writeData16(uint16_t data)
{
    _spiCSLow;
#ifdef _spiwrite24
    _spiwrite24(RA8875_DATAWRITE, data);
#else
	_spiwrite(RA8875_DATAWRITE);
	_spiwrite16(data);
#endif
    _spiCSHigh;
}

uint8_t ERDisplay::_readData(bool stat)
{
    uint8_t x;
    
    _spisetSpeed(SPI_SPEED_READ);
    _spiCSLow;
#ifdef _spixread
    if (stat == true) {_spixread(RA8875_CMDREAD, x);} else {_spixread(RA8875_DATAREAD, x);}
#else
    if (stat == true) {_spiwrite(RA8875_CMDREAD);} else {_spiwrite(RA8875_DATAREAD);}
    delayMicroseconds(50);                                                      // Stabilize time, else first bit is read wrong
    _spiread(x);
#endif
    _spiCSHigh;
    _spisetSpeed(SPI_SPEED_WRITE);
    return x;

}

uint8_t	ERDisplay::readStatus(void)
{
	return _readData(true);
}

void ERDisplay::writeCommand(const uint8_t d)
{
    _spiCSLow;
    _spiwrite16(RA8875_CMDWRITE+d);
    _spiCSHigh;
}

struct TouchCoord {
    x = -1;
    y = -1;
};
TouchCoord ERDisplay::readTouch() {
    coord xy;
    #if defined(ER_LCD_USE_GSL1680)
      int NBFinger = TS.dataread();
      if (NBFinger > 0) {
        xy.x = TS.readFingerX(0);
        yy.y = TS.readFingerY(0);
      }
    #endif
    return xy;
}