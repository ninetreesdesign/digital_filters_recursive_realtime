// create a general digital filter program to generate real-time y[n] = filterfctn( x[n] )
//  where filter is a[i]*x[i] + b[i]*y[i]  (A/B)
// start with IIR filters from S. Smith's DSP book
// add a steep BPF FIR using t-filter website tool
//
// to do: pass array pointers instead of needing globals
// better way to initialize b[i] without a 3rd array?
// add step function simulated input
// 9/26 why does output have repeating discontinuity? sample length affects it
// added BPF from SSmith
// TTD: add a 3 pt median filter, get the FIR filter to work
// median filter made; need to add to this

#define CONSOLE  Serial       // Debug console (Monitor)
#define P     CONSOLE.print        // print shortcut
#define Pln   CONSOLE.println      // print shortcut
#include <FlexiTimer2.h>
#include "ADC.h"
// #include <math.h>

//const byte BUF_LEN = 40;
//char input_buf[BUF_LEN];
const int ard_led_pin = 13;
const int noise_pin = A9;
const int signal_pin = A0;

const int NUM_SAMPLES = 100;
const int NUM_COEFFS = 20;    // of filter kernel numerator and denom.
double a[NUM_COEFFS];  // feedforward (past outputs)
double b[NUM_COEFFS];  // feedback    (past inputs)
double x[NUM_SAMPLES];
double y[NUM_SAMPLES];
int n = 0;  // index of where the next input and output value are written to
long int t0 = 0;
/*

      FIR filter designed with
      http://t-filter.appspot.com

      sampling frequency: 100 Hz

      0 Hz - 3 Hz
      gain = 0
      desired attenuation = -40 dB
      actual attenuation = -40.58001738138903 dB

      5 Hz - 25 Hz
      gain = 1
      desired ripple = 1 dB
      actual ripple = 0.7033983470148282 dB

      27 Hz - 50 Hz
      gain = 0
      desired attenuation = -40 dB
      actual attenuation = -40.58001738138903 dB

*/
#define FILTER_TAP_NUM 81
#define FILTER_TAP_NUM2 40

double d[FILTER_TAP_NUM2] = {
   0.027913521377381983,
  -0.000036248292707123435,
  -0.004506786872582961,
  0.022147269984827394,
  0.014340015199915871,
  -0.03007662651402011,
  -0.03144785854216077,
  0.005361160737041525,
  -0.017384022806819308,
  -0.08414624581737447,
   -0.06895441478626137,
  -0.08414624581737447,
  -0.017384022806819308,
  0.005361160737041525,
  -0.03144785854216077,
  -0.03007662651402011,
  0.014340015199915871,
  0.022147269984827394,
  -0.004506786872582961,
  -0.000036248292707123435,  
                  0.027913521377381983,
  -0.000036248292707123435,
  -0.004506786872582961,
  0.022147269984827394,
  0.014340015199915871,
  -0.03007662651402011,
  -0.03144785854216077,
  0.005361160737041525,
  -0.017384022806819308,
  -0.08414624581737447,
   -0.06895441478626137,
  -0.08414624581737447,
  -0.017384022806819308,
  0.005361160737041525,
  -0.03144785854216077,
  -0.03007662651402011,
  0.014340015199915871,
  0.022147269984827394,
  -0.004506786872582961,
  -0.000036248292707123435
};

double c[FILTER_TAP_NUM] = {
  0.007182971089445487,
  0.002894016726567146,
  0.0012868684668369329,
  0.-2005610561690641788,
  0.007641518409645428,
  -0.002588673497353906,
  -0.013320750427373716,
  -0.0083464981114087,
  0.0015182750959557181,
  -0.0028798566264304423,
  -0.012110754260514434,
  -0.007632629401413452,
  -0.00023648981345261686,
  -0.0067939327225860005,
  -0.01210006611114929,
  0.0002997702900595074,
  0.009805679132101723,
  -0.0005109545302410557,
  -0.004862609336274474,
  0.013467537453527816,
  0.022689035457715532,
  0.00643428761619743,
  0.0024331208003541428,
  0.024835006017480003,
  0.027913521377381983,
  -0.000036248292707123435,
  -0.004506786872582961,
  0.022147269984827394,
  0.014340015199915871,
  -0.03007662651402011,
  -0.03144785854216077,
  0.005361160737041525,
  -0.017384022806819308,
  -0.08414624581737447,
  -0.06895441478626137,
  -0.000128483007731669,
  -0.05094021101062967,
  -0.1784704722429822,
  -0.09629956147674594,
  0.23588399123639064,
  0.4347050408292225,
  0.23588399123639064,
  -0.09629956147674594,
  -0.1784704722429822,
  -0.05094021101062967,
  -0.000128483007731669,
  -0.06895441478626137,
  -0.08414624581737447,
  -0.017384022806819308,
  0.005361160737041525,
  -0.03144785854216077,
  -0.03007662651402011,
  0.014340015199915871,
  0.022147269984827394,
  -0.004506786872582961,
  -0.000036248292707123435,
  0.027913521377381983,
  0.024835006017480003,
  0.0024331208003541428,
  0.00643428761619743,
  0.022689035457715532,
  0.013467537453527816,
  -0.004862609336274474,
  -0.0005109545302410557,
  0.009805679132101723,
  0.0002997702900595074,
  -0.01210006611114929,
  -0.0067939327225860005,
  -0.00023648981345261686,
  -0.007632629401413452,
  -0.012110754260514434,
  -0.0028798566264304423,
  0.0015182750959557181,
  -0.0083464981114087,
  -0.013320750427373716,
  -0.002588673497353906,
  0.007641518409645428,
  0.005610561690641788,
  0.0012868684668369329,
  0.002894016726567146,
  0.007182971089445487
};
float adc_scale = 3.3 / 4096.0;
float t = 0;    // time (sec)
///////////////////////////////////////////////////////////////////////////////////
// filter_type options: NONE, LPF1, HPF1, LPF4, BPF1, NOTCH1, FIR1,FIR2
// signal_source: OPEN_PIN, V_A0, SQUARE, CUSTOM1
int f_sample = 100;  // sampling rate in Hz
double fc = 0.02;   // normalized to 0 - 0.5 freq range, freq = fc * f_sample
double BW = 0.1;    // where applicable
String filter_type = "FIR2";   
String signal_source = "SQUARE";
double freq_source = 9.0; // Hz
///////////////////////////////////////////////////////////////////////////////////

ADC *adc = new ADC(); // adc object

// ---------------------------------------------------------------------
void setup() {
  digitalWrite(ard_led_pin, 1); // show connect time with LED
  Serial.begin(115200); // the set rate doesn't matter for teensy 3
  while (!Serial && (millis() - t0 < 7000)) {  // continues if the monitor window is never opened
    ; // wait for serial port to connect. Needed for Leonardo & Teensyduino 3
  }
  digitalWrite(ard_led_pin, 0);
  //  Pln(F("Starting..."));
  delay(1000);

  for (int i = 0; i < NUM_SAMPLES; i++) {
    x[i] = 0;
    y[i] = 0;
  }
  for (int i = 0; i < NUM_COEFFS; i++) {
    a[i] = 0;
    b[i] = 0;
    c[i] = 0;
  }

  int sampling_interval = 1000 / f_sample; // in msec
  calc_coeffs(filter_type, fc, f_sample); // only at startup for now (not dynamically changed filter)

  adc->setConversionSpeed(ADC_HIGH_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_HIGH_SPEED);   // change the sampling speed
  adc->setAveraging(4); // set number of averages
  adc->setResolution(12); // set bits of resolution  (adc->getMaxValue(ADC_0) will give this value)
  // always call the compare functions after changing the resolution!
  adc->enableCompare(1.0 / 3.3 * adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
  // Pln(adc->getMaxValue(ADC_0));
  x[0] = 0.0 * ( adc->analogRead(signal_pin) ); // initial read ADC to get first point to work. Will overwrite on next ADC read
  filter_it();
  y[0] = calc_output();

  FlexiTimer2::set(sampling_interval, 1.0 / 1000, filter_it); // call every N 1ms "ticks"
  FlexiTimer2::start();
}

void loop() {
  // main
}

double calc_output () {
  double sum = 0;
  int nn;   // circular buffer index
  nn = n;
  // P("  nn="); Pln(nn);
  for (int i = 0; i < NUM_COEFFS; i++) {
    if (nn - i < 0) nn += NUM_SAMPLES;
    //   P("  i="); P(i);    P("  nn-i ="); P(nn - i);
    sum += a[i] * x[nn - i] + b[i] * y[nn - i];
  }
  //  Pln();
  return (sum);
}

void filter_it() {
  static char str[40] = "";
  // static int i = 0; P(i++);
  if (signal_source == "OPEN_PIN") {
    x[n] = adc_scale * adc->analogRead(noise_pin);   // get next input sample (random noise if no connection on pin)
  }
  else if (signal_source == "V_A0") {
    x[n] = adc_scale * float(adc->analogRead(signal_pin));
  }
  else if (signal_source == "SQUARE") {
    float f0 = freq_source; // something slow enough to see response
    x[n] =  sin(2 * M_PI * f0 * t) > 0.5 ? 1 : 0;     // 50% duty cycle
  }
  else if (signal_source == "PULSE") {
    float f0 = freq_source; // something slow enough to see response
    x[n] =  sin(2 * M_PI * f0 * t) > 0.95 ? 1 : 0;    // 5% duty cycle
  }
  else if (signal_source == "CUSTOM1") {
    float f0 = freq_source; // something slow enough to see response
    x[n] =  0.5 + (0.49 * ( 0.25 * sin(6.28 * 6 * t)) + 0.75 * sin(2 * M_PI * f0 * t));
  }

  y[n] = calc_output(); // get filtered output sample // input and output data arrays and coefficient arrays are global
  P(float(n) / NUM_SAMPLES / 10); P(", "); P(x[n]);  P(", ");  Pln(y[n]);
  //  sprintf(str, " % f", calc_output());
  //Pln(str);
  n++;
  if (n >= NUM_SAMPLES ) n = 0;
  t += 1.0 / f_sample;  // increment time by sample interval
}


void calc_coeffs(String filter_type, double fc, int f_sample) {
  // calculates coefficients for various filters
  // returns 0 if no error

  // set all coeffs to 0 first;
  for (int i = 0; i < NUM_COEFFS; i++) {
    a[i] = 0;
    b[i] = 0;
  }
  // the following 5 equations from Steven W. Smith's DSP, Ch 19: Recursive Filters (IIR)
  double x_decay = exp(-2 * M_PI * fc);
  if (filter_type == "NONE") {
    // returns the values unmodified
    a[0] = 1.0;
    b[0] = 0.0;
  }
  else if (filter_type == "LPF1") {
    a[0] = 1.0 - x_decay;
    b[1] = x_decay;
  }
  else if (filter_type == "HPF1") {
    a[0] = (1.0 + x_decay) / 2;
    a[1] = -(1.0 + x_decay) / 2;
    b[1] = x_decay;
  }
  else if (filter_type == "LPF4") {
    a[0] = pow((1.0 - x_decay), 4);
    b[1] =  4.0 * x_decay;
    b[2] = -6.0 * pow(x_decay, 2);
    b[3] =  4.0 * pow(x_decay, 3);
    b[4] = -1.0 * pow(x_decay, 4);
  }
  else if (filter_type == "BPF1") {
    // see pg. 326, 327
    double R;
    double K;
    R = 1 - 3 * BW;
    K = (1 - 2 * R * cos(2 * M_PI * fc) + R * R) / (2 - 2 * cos(2 * M_PI * fc));
    a[0] = 1 - K;
    a[1] = 2 * (K - R) * cos(2 * M_PI * fc);
    a[2] = R * R - K;
    b[1] = 2 * R * cos(2 * M_PI * fc);
    b[2] = -R * R;
  }
  else if (filter_type == "NOTCH1") {
  }
  else if (filter_type == "FIR1") {
    // all a's are zero for FIR
    for (int i = 0; i < FILTER_TAP_NUM; i++) {    // **************************
      a[i] = c[i];
    }
  }
  else if (filter_type == "FIR2") {
    // all a's are zero for FIR
    for (int i = 0; i < FILTER_TAP_NUM2; i++) {
      a[i] = d[i];
      // not B???    }
    }
  }
  // http://www.earlevel.com/main/2012/12/15/a-one-pole-filter/  this is equiv tl LPF1 above
  else if (filter_type == "EL_LPF") {
    b[1] = x_decay;  // = exp(-2 * M_PI * fc);
    a[0] = 1.0 - b[1];
  }
  // For a one-pole highpass response, just negate the lowpass calculation;
  // this inverts the spectrum, so you’ll need to flip the frequency range by subtracting Fc from 0.5
  else if (filter_type == "EL_HPF") {
    b[1] = -exp(-2 * M_PI * (0.5 - fc));
    a[0] = 1 + b[1];
  }
}


