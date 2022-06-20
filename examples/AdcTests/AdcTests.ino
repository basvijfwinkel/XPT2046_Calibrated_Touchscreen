#include <XPT2046_Touchscreen.h>
#include <SPI.h>

#define CS_PIN  8
// MOSI=11, MISO=12, SCK=13

# define ADC_VREF 2.5F

//XPT2046_Touchscreen ts(CS_PIN);
//#define TIRQ_PIN  2
XPT2046_Touchscreen ts(CS_PIN);  // Param 2 - NULL - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, 255);  // Param 2 - 255 - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

void setup() {
  Serial.begin(38400);
  ts.begin();
  ts.setADCVRef(ADC_VREF); // set the reference voltage for temperature measurements
  
  while (!Serial && (millis() <= 1000));
}

void loop() {
  Serial.print("VBat:");Serial.println(ts.getVBat());     // connected to GND on some boards
  Serial.print("AuxIn:");Serial.println(ts.getAuxIn());   // connected to GND on some boards
  Serial.print("Temp (C):");Serial.println(ts.getTemp()); // check what the VREF is on your board before calling this method.
  Serial.print("Temp (F):");Serial.println(ts.getTempF()); 
  Serial.println();

  delay(1000);
}
