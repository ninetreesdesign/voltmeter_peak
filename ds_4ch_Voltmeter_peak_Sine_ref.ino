/*
  use pair(s) of analog inputs to measure differential voltages
  added feature to read Vpeak

  added a reference sine wave output
  added a btn to scale it

  // broken; interrupt stops, working now; what fixed?

  // need to fix HW LPF (unground)

*/
#include <FlexiTimer2.h>

#include <U8g2lib.h>

//#ifdef U8X8_HAVE_HW_SPI
//#include <SPI.h>
//#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define P      Serial.print
#define BT     P    //mySerial.print
#define Pln    Serial.println
#define BTln   Pln //mySerial.println



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

#define DAC_RESOLUTION  12
const float twopi = 3.14159 * 2;
// Peak scale values
#define fourdBu    1.736
#define zerodBV    1.414
#define zerodBu    1.095
#define oneVpk     1.000
#define neg10dBV   0.447

const bool PRINT_FLAG = 1;
const int led_pin = LED_BUILTIN;      // pin for the LED
// encoder connection A C B; common to gnd; A,B with pullups
const byte pinA = 3;          // encoder pin, interrupt
const byte pinB = 5;          // encoder pin, interrupt
const byte pinC = 4;          // low = gnd for common
const byte shaftBtn = 2;
volatile int N = 0;
volatile int k_prev = 0;
volatile int k = 0;
volatile int i = 0;
int d;
int encStep = 0;
volatile int Q = 0;
long t_enc = 0;  // keep track of time since last encoder pulse

const float out_ampl[3] = {neg10dBV, oneVpk,  fourdBu};
const float v_ref = 3.297;
float dac_scale = out_ampl[1]/v_ref;
const float f = 100;                // Hz ************
const int points = 100;              // for ~64 or less, add HW LPF ~ 1kHz
const float tstep_f = 1.0/f/points;     // sec for interrupt duration
const int   tstep = (int)(tstep_f * 1.0e6);

float t = 0;
float val;
float wave;
uint8_t dac_scale_index = 1;

// OLED constants
int line_height = 16;
int char_width = 8;
int line_num;
char str[32] =  "\0";
char str1[80] = "\0";


const int ADC_BITS = 12;
const int ADC_MAX_VAL = pow(2, ADC_BITS);
// define ports for 4 diff pairs, avoiding I2C and SPI on Teensy 3.2
// 0:A0,A1  1:A2,A3  2:A6,A7  3:A8,A9
const byte adc_pair[4][2] = {
    {A0, A1},
    {A2, A3},   //***************
    {A6, A7},
    {A8, A9}
};

const float scale[4] {   // scale factor for range voltage dividers
    1.00,
    100 , // ******************
    1.00,
    1.00
};

int overrange_flag;  // for adc read
float v1;
float v2;
float v_diff[2];
float v_pk_max[2];
float v_pk_min[2];

elapsedMillis since_sample;    // timer to control when to measure again
elapsedMillis since_print;
elapsedMicros us_counter = 0;   // with tstep
elapsedMillis since_btn;
// todo: add btn read interval
const int NUM_READINGS = 4;

const uint16_t btn_interval =     15;       // ms
      uint16_t sample_interval =  20;
const uint16_t print_interval =  100;



float v_hist_max[4][16] = {
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}
};
float v_hist_min[4][16] = {
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}
};


// ISR
void updateDAC() {
    // pk-pk V, min at 0
    wave =  dac_scale * 0.5*(sin(twopi*f*t) + 1.0);  // 0 to 1.0 * scaling factor

    // conver [0 - 1] to [0 - 4095]
    // val = 4095 * wave;
    val = mapFloat(wave, 0,1, 0,4095)+.0001;
    analogWrite(A14, (int)val);   // hardware DAC
    t += tstep_f;
    if (t > 1e6) t = 0;
}


void setup(void) {
    pinMode(led_pin, OUTPUT);
    pinMode(shaftBtn, INPUT_PULLUP);
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, OUTPUT);
    digitalWrite(pinC, 0);
    attachInterrupt(pinA, updateEncoder, CHANGE);
    attachInterrupt(pinB, updateEncoder, CHANGE);
    //  attachInterrupt(shaftBtn, updateEncoder, CHANGE);

    analogReadResolution(ADC_BITS);
    analogWriteResolution(DAC_RESOLUTION);

    // i2cScan();
    u8g2.begin();

    delay(1000);
    u8g2.clearBuffer();          // clear the internal memory
    MED_FONT;;
    u8g2.drawStr((0 + 9) * char_width - 2, 1 * line_height - 2, "V"); //
    u8g2.drawStr((0 + 9) * char_width - 4, 2 * line_height - 2, "HV"); //

    Serial.begin(115200); // rate doesn't matter for teensy routed to USB console
    while (!Serial && (millis() < 5000)) {} // include timeout if print console isn't opened
    Serial.println(" USB Serial Print initialized*");
    Serial.print("t_step_f [ms]  " + String(tstep_f*1000.0,4));
    Serial.print("  ");
    Serial.println(tstep);

    // select font and size
    // u8g2_font_t0_17b_mn  // 14 pix height
    // u8g2.setFont(u8g2_font_6x12_mr);  // 7 pix
    //u8g2.setFont(u8g2_font_9x15B_mf);  // 11 pix
    //u8g2.setFont(u8g2_font_profont22_mf);  // 14 pix

    u8g2.clearBuffer();          // clear the internal memory
    MED_FONT;;
    u8g2.drawStr((0 + 9) * char_width - 2, 1 * line_height - 2, "V"); //
    u8g2.drawStr((0 + 9) * char_width - 4, 2 * line_height - 2, "HV"); //
    SMALL_FONT;
    sprintf(str,"%5.3f Vpk",out_ampl[dac_scale_index]);
    u8g2.drawStr((11) * char_width, 1 * line_height - 1, str); //
    u8g2.drawStr((11) * char_width, 2 * line_height - 1, " 90 Hz"); //
    u8g2.sendBuffer();          // transfer internal memory to the display


    FlexiTimer2::set(1, tstep_f, updateDAC); // call every 1 (n sec) "ticks"
    FlexiTimer2::start();

    delay(100);
}


void loop(void) {
    // create a reference sine wave on internal DAC output
    // use interrupt instead
    static int btn1;
    static int btn1_prev = 1;

    // mesaure two pairs on analog inputs
    int n_ch = 2;   // only using 2 of 4
    static int i_h = 0; // ring counter

    if (since_sample >= sample_interval) {
       // sample_interval = random(4,11);
        since_sample -= sample_interval;
        // read voltage for each channel
//     int x1 = analogRead(A0);
//     int x2 = analogRead(A1);
//     Serial.println(String(x1) + "    " + String(x2) );
//     Serial.println(String(float(x1)/ADC_MAX_VAL*v_ref) + "  " + String(float(x2)/ADC_MAX_VAL*v_ref) );

        v_pk_max[0] =-0.10;
        v_pk_max[1] =-0.20;
        v_pk_min[0] =999;
        v_pk_min[1] =999;
        for (int ch_num = 0; ch_num < n_ch; ch_num++) {
            v_diff[ch_num] = scale[ch_num] * getV(ch_num, NUM_READINGS); //(v2 - v1);
            v_hist_max[ch_num][i_h] = v_diff[ch_num];
            v_hist_min[ch_num][i_h] = 0;//v_diff[ch_num];
        }
        // write value to v history array; use history to find a running peak value
        for (int ch_num = 0; ch_num < n_ch; ch_num++) {
            for (int i = 0; i < 16; i++ ) {
                if (v_hist_max[ch_num][i] > v_pk_max[ch_num])
                    v_pk_max[ch_num] = v_hist_max[ch_num][i];
                if (v_hist_min[ch_num][i] < v_pk_min[ch_num])
                    v_pk_min[ch_num] = v_hist_min[ch_num][i];
            }
        }
        i_h ++;
        i_h = i_h * (i_h < 16); // range from 0 to n;

//        for (int i = 0; i<16; i++) {
//            Serial.print(" " + String(v_hist_max[0][i]) );
//        }
//        Serial.println();

        if (since_btn >= btn_interval) {
            since_btn -= btn_interval;

            // check new btn for output scale select
            btn1 = digitalRead(shaftBtn);
            if (btn1 == 0 && btn1_prev == 1) {        // if pressed, loop through options
                digitalWrite(led_pin,1);
                delay(20);
                digitalWrite(led_pin,0);
                delay(5);
                dac_scale_index++;
                if (dac_scale_index > 2) dac_scale_index = 0;
                dac_scale = out_ampl[dac_scale_index]/v_ref;
                printOLED();
                if (PRINT_FLAG) printMONITOR();
            }
            btn1_prev = btn1;
        }

        if (since_print >= print_interval) {
            since_print -= print_interval;
            printOLED();
            if (PRINT_FLAG) printMONITOR();
        }
        delay(1);
    }
}

float getV(int ch_pair, int num_readings) {
    // read two ADC channels and find difference (Both channels must see a positive voltage)
    int32_t k1 = 0;
    int32_t k2 = 0;
    int32_t k;    // intermediate
    int32_t k_max = 0;
    int32_t k_min = 1e6;
    for (int i = 0; i < num_readings; i++) {
        k1 += 0; //analogRead(adc_pair[ch_pair][0]);
        k2 += analogRead(adc_pair[ch_pair][1]);
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

    return (float)(k2 - k1) * v_ref / ADC_MAX_VAL;
}


void printOLED() {
    byte num_dec = 2;
    // print to OLED display
    dtostrf(v_pk_max[0], 6, num_dec, str);
    LARGE_FONT;
    u8g2.drawStr(0 * char_width, 1 * line_height - 1, str); //
//    SMALL_FONT;
//    u8g2.drawStr((0 + 10) * char_width, 1 * line_height - 2, "V sig"); //
//
    dtostrf(v_pk_max[1], 6, num_dec, str);
    LARGE_FONT;
    u8g2.drawStr(0 * char_width, 2 * line_height + 0, str); //
//    SMALL_FONT;
//    u8g2.drawStr((0 + 10) * char_width - 4, 2 * line_height - 2, "HV"); //
//    //  SMALL_FONT;
//  //  u8g2.drawStr((0 + 11) * char_width - 2, 2 * line_height - 2, "amp"); //

    SMALL_FONT;
    sprintf(str,"%4.2f pp",out_ampl[dac_scale_index]);     // %5.3f
    u8g2.drawStr((11) * char_width, 1 * line_height - 1,  str); //
    u8g2.drawStr((11) * char_width, 2 * line_height - 1, "100 Hz"); // todo: show actual frequency
    u8g2.sendBuffer();          // transfer internal memory to the display
}


void printMONITOR() {
//    sprintf(str1, "max    %7.3f V_sig    %6.2f HV [amp] \t%4.0fHz   %5.3f \n", v_pk_max[0], v_pk_max[1], f, out_ampl[dac_scale_index]);
//    P(str1);
//    sprintf(str1, "min    %7.3f V_sig    %6.2f HV [amp] \t%4.0fHz   %5.3f \n", v_pk_min[0], v_pk_min[1], f, out_ampl[dac_scale_index]);
//    Pln(str1);
    sprintf(str1, "dif    %7.3f V_sig    %6.2f HV [amp] \t%4.0fHz   %5.3f \n", v_pk_max[0]-v_pk_min[0], v_pk_max[1]-v_pk_min[1], f, out_ampl[dac_scale_index]);
    P(str1);
}


// ---------------------------------------------------------------------------------
void updateEncoder() {
    //TEMP set to 1 for mech enc
    int div_N = 1;  // slow down data output rate; 8 gives enough resolution
    bool    b = digitalRead(shaftBtn);
    k = digitalRead(pinA);
    d = digitalRead(pinB);
    if (k_prev > k) { // rising edge
        t_enc = millis();
        i = -1 * (-1 + 2 * d); // flip dir
        N = N + i * !(Q % div_N);
        Q++;
        // modify to only print +1 or -1
        if (i >= 0) P(" "); // align print with leading sp for pos
        if (Q % div_N == 0) {
            Pln(i); // P(" v ");   if (N >= 0) P(" "); P(N);
            // P("\n");
            //  BT(i);
            //  BT("\n");
        }
    }
    // show interrupt occurring
    noInterrupts();
    digitalWrite(led_pin,1);
    delay(20);
    digitalWrite(led_pin,0);
    delay(5);
    interrupts();
    k_prev = k;
}


/// map one numerical span to another with floating point values
float mapFloat (float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



