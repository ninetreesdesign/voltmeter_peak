/*
  use pair(s) of analog inputs to measure differential voltages
  added feature to read Vpeak
*/
#include <U8g2lib.h>

//#ifdef U8X8_HAVE_HW_SPI
//#include <SPI.h>
//#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define P Serial.print

//#define SMALL_FONT u8g2.setFont(u8g2_font_6x12_mr);  // 7 pix
//#define MED_FONT   u8g2.setFont(u8g2_font_9x15B_mf);  // 11 pix
//#define LARGE_FONT u8g2.setFont(u8g2_font_profont22_mn); // u8g2.setFont(u8g2_font_t0_17b_mn); //u8g2.setFont(u8g2_font_profont22_mf);  // 14 pix
#define LARGE_FONT u8g2.setFont(u8g2_font_t0_22b_mn);
#define MED_FONT   u8g2.setFont(u8g2_font_t0_15b_mf );
#define SMALL_FONT u8g2.setFont(u8g2_font_t0_12b_mf);

// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// each of these gives same display
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, 19, 18, 20); // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0,  U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED


int led_pin = LED_BUILTIN;      // pin for the LED
// OLED constants
int line_height = 16;
int char_width = 8;
int line_num;
char str[22] =  "\0";
char str1[40] = "\0";

const uint16_t sample_interval = 10;
const uint16_t print_interval =  300;
const int ADC_BITS = 16;
const int ADC_MAX_VAL = pow(2, ADC_BITS);
// define ports for 4 diff pairs, avoiding I2C and SPI on Teensy 3.2
// 0:A0,A1  1:A2,A3  2:A6,A7  3:A8,A9
const byte adc_pair[4][2] = {
  {A0, A1},
  {A2, A3},
  {A6, A7},
  {A8, A9}
};

const float scale[4] {   // scale factor for range voltage dividers
  0.10,
  0.01,
  1.00,
  1.00
};
const float Vref = 3.3; // volts (supply ref)

int overrange_flag;  // for adc read
float v1;
float v2;
float v_diff[4];
float v_peak[4];

elapsedMillis since_sample;    // timer to control when to measure again
elapsedMillis since_print;


void setup(void) {
  pinMode(led_pin, OUTPUT);

  analogReadResolution(16);

  // i2cScan();
  u8g2.begin();

  Serial.begin(115200); // rate doesn't matter for teensy routed to USB console
  while (!Serial && (millis() < 5000)) {} // include timeout if print console isn't opened
  Serial.println(" USB Serial Print initialized*");


  // select font and size
  // u8g2_font_t0_17b_mn  // 14 pix height
  // u8g2.setFont(u8g2_font_6x12_mr);  // 7 pix
  //u8g2.setFont(u8g2_font_9x15B_mf);  // 11 pix
  //u8g2.setFont(u8g2_font_profont22_mf);  // 14 pix

  u8g2.clearBuffer();          // clear the internal memory
}

const int NUM_READINGS = 8;

void loop(void) {
  int n_ch = 2;   // only using 2 of 4
  static int i_h = 0; // ring counter
  if (since_sample >= sample_interval) {
    since_sample -= sample_interval;
    // read voltage for each channel
    for (int ch_num = 0; ch_num < n_ch; ch_num++) {
      v_diff[ch_num] = scale[ch_num] * getV(ch_num, NUM_READINGS); //(v2 - v1);
    }
  
    if (since_print >= print_interval) {
      since_print -= print_interval;
      printOLED();
      printMONITOR();
    }
  }
}


float getV(int ch_pair, int num_readings) {
  // read two ADC channels and find difference (Both channels must see a positive voltage)
  // this takes about 1.59ms for n=64, 6.35ms for n=256

  int32_t k1 = 0;
  int32_t k2 = 0;
  int32_t k;    // intermediate
  for (int i = 0; i < num_readings; i++) {
    k = analogRead(adc_pair[ch_pair][0]);
    k1 += k;
    k = analogRead(adc_pair[ch_pair][1]);
    k2 += k;
    delayMicroseconds(2);
  }
  k1 /= num_readings;
  k2 /= num_readings;

  if (k2 > (ADC_MAX_VAL - 2))
    overrange_flag = 1;
  else if (k1 > (ADC_MAX_VAL - 2))
    overrange_flag = -1;
  else
    overrange_flag = 0;

  return (float)(k2 - k1) * Vref / ADC_MAX_VAL;
}


void printOLED() {
  byte num_dec = 1;
  // print to OLED display
  dtostrf(v_diff[0], 6, num_dec, str);
  LARGE_FONT;
  u8g2.drawStr(0 * char_width, 1 * line_height - 1, str); //
  MED_FONT;
  u8g2.drawStr((0 + 10) * char_width, 1 * line_height - 2, "V sig"); //

  dtostrf(v_diff[1], 6, num_dec, str);
  LARGE_FONT;
  u8g2.drawStr(0 * char_width, 2 * line_height + 0, str); //
  MED_FONT;
  u8g2.drawStr((0 + 10) * char_width - 4, 2 * line_height - 2, "V"); //
  //  SMALL_FONT;
  u8g2.drawStr((0 + 11) * char_width - 2, 2 * line_height - 2, "amp"); //

  u8g2.sendBuffer();          // transfer internal memory to the display
}


void printMONITOR() {
  sprintf(str1, "\t%6.2f V_sig    %6.1f V_amp  \n", v_diff[0], v_diff[1] );
  P(str1);
}






