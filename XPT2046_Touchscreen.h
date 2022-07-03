/* Touchscreen library for XPT2046 Touch Controller Chip
 * Copyright (c) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _XPT2046_Touchscreen_h_
#define _XPT2046_Touchscreen_h_

#include "Arduino.h"
#include <SPI.h>

#define XPT2046_NO_IRQ 255
#define ADC_VREF 2.5F	
	

#if defined(__IMXRT1062__)
#if __has_include(<FlexIOSPI.h>)
	#include <FlexIOSPI.h>
#endif
#endif

#if ARDUINO < 10600
#error "Arduino 1.6.0 or later (SPI library) is required"
#endif

class TS_Point {
public:
	TS_Point(void) : x(0), y(0), z(0), calx(0), caly(0) {}
	TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
	TS_Point(int16_t x, int16_t y, int16_t z, int16_t calx, int16_t caly) : x(x), y(y), z(z), calx(calx), caly(caly) {}
	bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
	bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
	int16_t x, y, z;
	int16_t calx, caly;
};

class XPT2046_Touchscreen {
public:
	constexpr XPT2046_Touchscreen(uint8_t cspin = 8, uint8_t tirq=XPT2046_NO_IRQ) : csPin(cspin), tirqPin(tirq) { }
	void setCalibration(int16_t hmin,int16_t hmax,int16_t vmin,int16_t vmax,int16_t hres,int16_t vres,int16_t xyswap);
	int16_t calibratedCoord(int16_t raw, int16_t axis);
	int16_t remap(int16_t min, int16_t max, int16_t res, int16_t dotoffset, int16_t returnfield);

	bool begin(SPIClass &wspi = SPI);
#if defined(_FLEXIO_SPI_H_)
	bool begin(FlexIOSPI &wflexspi);
#endif

	TS_Point getPoint(bool doUpdateFirst = true);
	bool tirqTouched();
	bool touched();
	void readData(int16_t *x, int16_t *y, int16_t *z);
	bool bufferEmpty();
	uint8_t bufferSize() { return 1; }
	void setRotation(uint8_t n) { rotation = n % 4; }
	void setADCVRef(float vref) { adc_vref = vref; }
	float getVBat();
        float getAuxIn();
        float getTemp();
        float getTempF();
        float getADCVRef();
	
	volatile bool isrWake=true;

private:
	void update();
	int16_t updateADC(int16_t adc);
	uint8_t csPin, tirqPin, rotation=1;
	int16_t xraw=0, yraw=0, zraw=0, xcal=0, ycal=0;
	uint32_t msraw=0x80000000;
	SPIClass *_pspi = nullptr;
	int16_t cal_hmin   = 0;
        int16_t cal_hmax   = 4095;
        int16_t cal_vmin   = 0;
        int16_t cal_vmax   = 4095;
        int16_t cal_hres   = 320;
        int16_t cal_vres   = 240;
        int16_t cal_xyswap = 0;
        float adc_vref     = ADC_VREF;

#if defined(_FLEXIO_SPI_H_)
	FlexIOSPI *_pflexspi = nullptr;
#endif
};

#ifndef ISR_PREFIX
  #if defined(ESP8266)
    #define ISR_PREFIX ICACHE_RAM_ATTR
  #elif defined(ESP32)
    // TODO: should this also be ICACHE_RAM_ATTR ??
    #define ISR_PREFIX IRAM_ATTR
  #else
    #define ISR_PREFIX
  #endif
#endif

#endif
