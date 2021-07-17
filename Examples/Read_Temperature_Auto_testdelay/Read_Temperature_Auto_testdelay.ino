/**************************************************************************
 * Test autoconversion delay requirement
 **************************************************************************/

#include <SPI.h>
#include <MAX31865.h>

#define RTD_CS_PIN   10

MAX31865_RTD rtd( MAX31865_RTD::RTD_PT100, RTD_CS_PIN );

float t[100];
int indx=0, my_delay = 30;


void setup()
{
  Serial.begin( 115200 );

  /* Initialize SPI communication. */
  SPI.begin( );

  /* Allow the MAX31865 to warm up. */
  delay( 100 );

  // turn on vbias
  // auto convert
  // set wire
  rtd.configure_control( 
    true,     // V_BIAS enable
    true,     // auto conversion
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

}

void loop() 
{
  //Read the MAX31865 rtd register
  bool status = rtd.read_rtd( );

  // clear faults
  if (status == true) { 
    Serial.println(" RTD fault occured " );
    rtd.clearFaults(); 
  }

  t[indx] = rtd.ohmsX100_to_celsius(rtd.resistance()*100.);
  indx = indx + 1;
  if (indx == 100) {
    indx=0;
    
    float sum = 0., mean, std=0.;
    for (int i=0; i<100; i++){
      sum += t[i];
    }
    mean = sum/100.;
    for (int i=0; i<100; i++){
      std += pow(t[i]-mean,2);
    }
    std = sqrt(std/100.);
    
    Serial.printf("Delay: %d, Average: %f, STD: %f\n", my_delay, mean, std);

    if (my_delay > 0) {my_delay -= 1;}
    else my_delay =30;
    
  }
 
  delay( my_delay );  // 17ms for 60Hz, 20ms for 50Hz
}
