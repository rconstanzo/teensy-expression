//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
 /*
 *  USB-MIDI high resolution (14-bit) expression pedal.
 *  by Rodrigo Constanzo, http://www.rodrigoconstanzo.com // rodrigo.constanzo@gmail.com
 *  
 *  coded for
 *     Dunlop DVP4: https://www.jimdunlop.com/product/dvp4
 *     Teensy 3.2: https://www.pjrc.com/store/teensy32.html
 *     Adafruit ADS1115: https://www.adafruit.com/product/1085
 *  
 *  EXPLANATION
 *  -----------
 *  The code takes analog readings from the ADS1115 external ADC and scales, smooths, and
 *  constrains the output before sending it as high resolution MIDI CCs (using two CCs as
 *  MSB and LSB). The code includes a calibration routine that takes the minimum and maximum
 *  readings of the pedal and stores them in the internal EEPROM. To use the calibration
 *  routine, you must send Program Change message 13 followed by 69, both on channel 11. The
 *  initial state of the device can be reset by sending Program Change message 10 followed by 
 *  110.
 *  (example Max/MSP code below)
 *  
 *  last update on 13/9/2019
 */
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// required libraries
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <ResponsiveAnalogRead.h>
#include <EEPROM.h>


// declared variables

// define MIDI channel
const int midi_channel = 11;

// define CC for the first CC (MSB)
const int cc_msb = 1;

// define CC for the second CC (LSB)
// **this should be 32 higher than the MSB if you want it to work properly in apps that accept 14-bit MIDI**
const int cc_lsb = 33;

// internal LED for calibration/reset status
const int led = 13;

// declare the available sensor range
// using a TeensyLC and ADS1115 gives you 80% of 4.096v (3.3v is 80%) so that makes 26400 80% of 32768
const int sensor_range = 26400;

// set output range (14-bit = 16383)
const int output_range = 16383;

// ints for filtering repeats
int current_value = 0;
int previous_value = 0;

// flag to check if in calibration mode
int calibrate_flag = 0;

// flag to check if in reset mode
int reset_flag = 0;

// store whether device has ever been calibrated in order to allow default settings
int has_been_calibrated = EEPROM.read(10);

// by default leave 5% slop at the top end and 2.5% slop at the bottom
int calibrate_min = 0 + (sensor_range / 40);
int calibrate_max = sensor_range - (sensor_range / 20);

// instantiate adafruit library
Adafruit_ADS1115 ads;

// variable to store sensor reading
int16_t adc0;

// initialize sensor reading
ResponsiveAnalogRead analog(0, true);


/////////////////////////// SETUP ///////////////////////////

void setup(void) 
{

  // Serial.begin(9600);

  // look for program change messages (for calibration routine)
  usbMIDI.setHandleProgramChange(OnProgramChange);

  // if the device has been calibrated, use the stored settings instead of default settings
  if (has_been_calibrated == 1) {
    calibrate_min = constrain((BitShiftCombine(EEPROM.read(0), EEPROM.read(1))), 0, sensor_range);
    calibrate_max = constrain((BitShiftCombine(EEPROM.read(2), EEPROM.read(3))), 0, sensor_range);
  }

  /*
  Serial.print("The initial calibrate_min value is");
  Serial.print(" ");
  Serial.println(calibrate_min);

  Serial.print("The initial calibrate_max value is");
  Serial.print(" ");
  Serial.println(calibrate_max);
  */

  // initialize ADS1115 library
  ads.begin();

  // set ADS1115 gain to 1x, to get the maximum possible range with the 3.3v Teensy
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  // set smoothing resolution
  analog.setAnalogResolution(sensor_range);

  // reset sketch to initial state
  // EEPROM.write(10,255);

}


/////////////////////////// LOOP ///////////////////////////

void loop(void) {
  
  // read analog pin from ADS1115
  adc0 = ads.readADC_SingleEnded(0);
  analog.update(adc0);

  // scale the available reading range to the available output range (typically 14-bit MIDI)
  current_value = map(analog.getValue(), calibrate_min, calibrate_max, 0, output_range); 

  // constrain value after scaling to avoid exceeding the available range
  current_value = (constrain(current_value, 0, output_range));

  // filter repeats
  if (current_value != previous_value) {
      if (current_value >> 7 != previous_value >> 7) {
        usbMIDI.sendControlChange(cc_msb, current_value >> 7, midi_channel);
      }
      usbMIDI.sendControlChange(cc_lsb, current_value & 127, midi_channel); 
      previous_value = current_value;
  }

  // update MIDI
  usbMIDI.read();

  // flash the onboard LED for 100ms every 5 seconds, for troubleshooting purposes
  /*
  static long prevT = 0;
  static const long blinkT=5000;

  unsigned long currT = millis();
  digitalWrite(led, (currT-prevT)>=blinkT ? HIGH : LOW );
  if(currT-prevT>=(blinkT+100)) prevT=currT; 
  */
  
}


/////////////////////////// FUNCTIONS ///////////////////////////

// create a single 16-bit int from two 8-bit ints
int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  int combined; 
  combined = x_high;              //send x_high to rightmost 8 bits
  combined = combined<<8;         //shift x_high over to leftmost 8 bits
  combined |= x_low;                 //logical OR keeps x_high intact in combined and fills in                                                             //rightmost 8 bits
  return combined;
}

// calibration and reset routines to run when receiving program change messages on channel 11
void OnProgramChange(byte channel, byte program) {
  // offset midi program channel message since they count from 1 instead of 0
  program++;
  if (channel == 11) {
    if (program == 13) {
      calibrate_flag = 1;
      } else if (calibrate_flag == 1) {
         if (program == 69) {

          // enable LED notification of status
          digitalWrite(led, HIGH);
          // Serial.println("calibrating!");
          
          while (calibrate_flag == 1) {
            
            // actual calibration function 
            calibrateSensor();

            // terminate while loop
            calibrate_flag = 0;      
          }

          // turn off notification LED
          digitalWrite(led, LOW);

          // add a little buffer at each extreme to ensure the values can achieve the full range
          calibrate_min += 5;
          calibrate_max -= 20;
            
          // clamp and slightly limit reading values and write new min/max values to EEPROM
          EEPROM.write(0, (constrain(calibrate_min, 0, sensor_range / 5)) >> 8);
          EEPROM.write(1, (constrain(calibrate_min, 0, sensor_range / 5)) & 255);
          EEPROM.write(2, (constrain(calibrate_max, sensor_range - sensor_range / 5, sensor_range) - 10) >> 8);
          EEPROM.write(3, (constrain(calibrate_max, sensor_range - sensor_range / 5, sensor_range) - 10) & 255);
          
          // write the fact that device has been calibrated to EEPROM
          EEPROM.write(10, 1);

          /*
          Serial.println("done calibrating!");
          Serial.println(calibrate_min);
          Serial.println(calibrate_max);
          */

         } else {
          calibrate_flag = 0;
         }
    }
  }
    // reset initial state
    if (channel == 11) {
      if (program == 10) {
        reset_flag = 1;
        } else if (reset_flag == 1) {
          if (program == 110) {

          digitalWrite(led, HIGH);
          delay(100);
         
          // write the fact that device has been reset to EEPROM
          EEPROM.write(10, 255);

          // reset minimum and maximum to initial values
          calibrate_min = 0 + (sensor_range / 40);
          calibrate_max = sensor_range - (sensor_range / 20);

          // Serial.println("values reset!");
          
          // turn off notification LED
          digitalWrite(led, LOW);

          } else {
            reset_flag = 0;
          }
    }
  }
}

void calibrateSensor() {
  // declare variables to use in calibration routine
  bool calib_bool = true;
  bool schmitt = false;
  int schmitt_count = 0;

  // set minimum and maximum to absurd values
  calibrate_min = 1000000;
  calibrate_max = -100;
  
  while(calib_bool) {
    adc0 = ads.readADC_SingleEnded(0);
    analog.update(adc0);
    int reading = analog.getValue();
    
    // Serial.println("current reading value:");
    // Serial.println(reading);

    // update the minimum and maximum values
    if (reading < calibrate_min) {
      calibrate_min = reading;
    }
    if(reading > calibrate_max) {
      calibrate_max = reading;
    }

    // use a schmitt trigger to count how many times the full range of the pedal has been moved
    if (reading > sensor_range - (sensor_range / 4) && schmitt == false) {
      schmitt = true;
    }
    if (reading < sensor_range / 4 && schmitt == true) {
      schmitt = false;
      schmitt_count++;
    }

    // stop calibration routine after 4 passes have been made
    if(schmitt_count >= 4) {
      calib_bool = false;
    }
  }
}


//////////// EXAMPLE MAX CODE //////////////////////
/*

<pre><code>
----------begin_max5_patcher----------
2320.3ocyZk0iihjD94p9UvXsOTUI2tySRX0rR6gVMuLilG5UZ0noW0JMNsa
llCKN5pp4n+sO4AfAefSiwrcK0T.NSx3Khu3HC32t+tYKSeQjOy4u57yN2c2
uc+c2ouk5F2Uc8cyh4uDDwy0CaVPZbrHoX1byuUHdoPe+OFt4iNYh7znxhvz
Dm0Qo7lQkTFmVVDIJzOCP0cCWomY5xe4Mdv5gtkWD7wvjMeHSDTXDLDCtfN2
A6hV.l6.gT0ePfE.m+2tmeXR8iGpt2eb+8pCyuRTkjFlKbRSda550NAkEERI
qdPQgIhfzxD8HQ1hTleOH0EpgHECT.lQTWfwSAPyC3QBmhTGvB0Q3BqMcLlE
lNHSAEezzY4fj2rLrv4y7nRg8XgddrfXXEJLGmFrn.R9GCWW3Dmu7sQ4KcBS
jFItStTBklsJrtuKnTdVJxtR5JtGMB1WySQH8eftdSGeMSroLhm47.PSWQrG
chCWE1B9WItA8faiiIx0WCavMvMcUHO53rV3gxpq2NYMiGKJDYePjvWFIZCN
yyn30sBCHlYhP2HymFmFyKAntfLID93zbIiObSR35v.dRgijdec1S29hRQPP
clkpvtSHMNRvGcj1WLLHxsER8PSGRWkweVFw5yRuyXgiJcSqTomGTn9xZh07
ThI8BD4OcQlWySBd0Yqzvk6rgGlrOvFlIj1WMBXnFetDr1DBFeSXbYTQXdT3
pcIO5JxniXf.CJFzr4x+2H44hBonKkEyupgFrExBRSJVwK3siCdZMDwW6U6Z
X7joweVZukYkt3hnnd8U3ADefEebo2xTAxmg0Fap6fM1K4RmiFAecZVLWiR2
yBerq1bRMvGMEVy8KnRWgQmM1LP+aROVaOpN+Ckos1FN735eeoVazTZs8fFy
LFcqr1sqsRaLkh91xh1AUNaIWD5MpjqZ3yZp1Zrq4JQ7rDAGv0eRUD8Bqq4j
zPgGNHqBPSP8aiQiY95xj.kqbdCAPlRYMOPbZq+w7eAsmdd3upmtlwdtnXdH
s2MjRpgdK2ijhDIWRuL+irt6.3FU9RVZri3ksxXb4p.baEqZ4aLDUCCMXUig
riMt7Sklo6djOarPBbjhEdZ0fpMC2nHeWJXA2NvhqJKC69UBXw92bK6sDrGO
ntx014A1BhbCenm1l97CnELnLFj5+j2+94O7FHks3onzMOHYcLHE5+3iOdzQ
9veYM7InL70i6O7GeR8aeyeC73QKKhXaNE4lSOYRkYcBFTGWA1abk58A.bMU
Lq6ar6+mC4JC0Jruu339pTDI2vqpeEHy13YS21dUaxcYFuv9Nqh6qOhHfV1q
.B0c7ARrLAGei3vtDqegB.qK6ACrfgdLD5oAEivrIB.ZLQHVx48sFfH+ABPS
HtJ.RHiO.OQHtu8acX1iN5IQWXRgUIr7Z1P1DAvsaTfRxRc96pPUNue1+QHR
xe04e2T216mYseHBe9FMvpdOMHzzARdvmbVKc5s2ThFFQsdmU9U6yZxfnJm3
WrtbD3oYpptEqRTc31qsMwXkNfZpGwbDMf7hCQ6H90U7fuXMeE1We6oPcORb
MM0lPqOdyskO8Eq4oPx4LjCw.Z.N0DLhQMuEtow.dJO3vjOYuVgcCUJUrZug
xpGw5ueqCzE6gsusJPnssUwd8Rc+SglcePlRu8g7pDfeE8pDpaMwM7UIbBpy
uaMmw2tZZtXBCp5cExPSIi4DpifBY9SGnyNGjyWgvXqWpJArpxA2uBhuXzJX
7knVf2H5RkKBZLTK54pqXZuuMM8xqteWcUdZYVPMLpeCUN6DgUh7hvDt9yBY
2fTkP1ZPwgq1lJ0AUKop7W0mtjOQmUg42bUyLRyTw0TAkNpI0VgcO433Bqpf
P0fttUx2F0BdDVI0FkmlURUf040dxBNjQOFYCm8KM3vkFdcKMxFFtKXDTvV4
Ksu7bZmIUoHHulqFaEicRK331Dz0YSfVrz6IeiDpI1vDonwvcyFPtm3bBx.w
27Fw.MWM1NnTahrRFiHqDaTK05tqZkrIDNYLVn5LN8CIhIv5UsRLaWnq1JYS
VI0mS4oIuXytsgLyWPGt4hQOBuEhJ1eLJJvFBEXpz9H5wiOdkkXYCW1eDnxX
eKWntJSSAu7sa+rHKuZv50PtMfeIUq.7lquLLwbotf7YYhOGVOdy.3Yx5yKj
EmWlYp39EWSGnlEmJUkIkgUTSI5jKYdwqQ6Wj8th0KWEl9tBdQY9G9AQRoot
cInVyk6quqlX4l0gQQAoQFw6tleQtckpsXLy7qyqtcyXUaOG4SfPOoZYAFhY
HW8YxSnzV0sUMGX8jHThO.oFJwEwHT8YdHLEqNCr2zP6VK.z2rB.eO.wbl7V
vN8HPpKUec855BzMLs5tayR2llUaRkS0uY7kEoax3qBEl27olyZLwF0somFx
eUq105jJcptQI0yoMwqwX78kA79L.7f.4CtiREijJHM5HTIR0mwL2qcuPDQh
38mJyy0koUnd.nmm4L4SAg5N2b4j0eQMcmMB3ZVOnKCYrFd..jzcxp8wpm2G
BSTDeQiEB6ifZKDpwDiXdDbWKjUjtFCxI3cVxE5x77Z3adL.0yyb1AB39LO2
Zyfq5eLyYlmzMg30nl5fYe.Axz7BOeJgwL1VjuwdSca0jrVyhvnPrdDHerbh
FqCTZn6ZSZuceY7tebqHw4c7jbm2IhCWlFsZVcjuqzg3Sl9J9eUg65ywnEJf
UsAb2gQRVh4u3p5fyk4eZnMMbcVECzJWDv7VG55SbZ.qKSYz.rt2MVp4k7GY
bYje6XPJeZBEN11gNuI7iKXCQqNzHMshYT6ye3Y8Dyf4oxnM+nmc3z9pLGWW
uC6rhltSZM6RayZUOfDyX+wjYY5.6+LpTT2AU6SCCYtDSv9FAzWcK3MP.yRe
N4hkvcxEsw2Deijv+0q7KW.AHYlJSPRFFiL9NHHT80sL1B32kIDCPB0ZOr5e
l5dTkHL9B2OIhhRe9hkNOYlZlIxAzCpiuQkZT5gotq64+Oq65O0G.7FK.n+f
L+NYE.Wr3qnh.s4mJSVR0HA1NZ1nHfYlBJTRX+6z4xjsiULkKw0E6WwhYUkf
IsG9CmOa1Bo9UFoPT91pOca8aV59+39+DFSOkWB
-----------end_max5_patcher-----------
</code></pre>


*/
