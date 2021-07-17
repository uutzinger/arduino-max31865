/**************************************************************************
 * Test delay requried between conversions 
 **************************************************************************/

#include <SPI.h>
#include <MAX31865.h>

#define RTD_CS_PIN   10

MAX31865_RTD rtd( MAX31865_RTD::RTD_PT100, RTD_CS_PIN );
int my_delay = 70;
int indx = 0;
float t[10];

void setup()
{
  Serial.begin( 115200 );

  /* Initialize SPI communication. */
  SPI.begin( );

  /* Allow the MAX31865 to warm up. */
  delay( 100 );
  Serial.println("Setting up.");

  // turn off vbias
  // no auto convert
  // set wire
  rtd.configure_control( 
    false,     // V_BIAS enable
    false,     // auto conversion
    false,     // 1-shot, start conversion when CS goes high 
    false,     // 3-wire enable
    MAX31865_FAULT_DETECTION_NONE, // fault detection 
    false,      // fault status auto clear
    false);     // true = 50Hz filter, false = 60Hz

  // clear faults
  rtd.clearFaults(); 

  /* Set Threshold:
   * Low Thershold and High Threshold
	 */

  rtd.configure_thresholds(
    0x0000,    // Low Thresh 0x0000
    0xffff );  // High Thresh 0xffff
    // (uint16_t)((deg_C + 256.) * 32.) << 1

  Serial.println("Testing.");

}

void loop() 
{

  //rtd.clearFaults();     // clear faults
  rtd.enableBias(true);  // enable V_BIAS
  delay(10);             // wait until RC network has setteled, settles in 0-1ms
  rtd.oneShot();         // trigger measurement
  delay(my_delay);             // wait until conversion is complete

  uint8_t status = rtd.read_all( );

  rtd.enableBias(false);  // disable V_BIAS

  if (status != 0) { rtd.clearFaults(); }

  t[indx] = rtd.temperature();
  indx = indx + 1;
  if (indx == 10) {
    indx=0;
    
    float sum = 0., mean, std=0.;
    for (int i=0; i<10; i++){
      sum += t[i];
    }
    mean = sum/10.;
    for (int i=0; i<10; i++){
      std += pow(t[i]-mean,2);
    }
    std = sqrt(std/10.);
    
    Serial.printf("Delay: %d, Average: %f, STD: %f\n", my_delay, mean, std);
    /*
    for (int i=0; i<10; i++){
      Serial.print(t[i]);
      Serial.print(" ");
    }
    Serial.println();
    */
    if   ( my_delay > 0   ) { my_delay = my_delay - 1; }
    else                    { my_delay = 70; }
  }
  
}
