/*
 * DHT22.c
 *
 * Created: 9/22/2016 09:52:49
 *  Author: ROBERTOC
 */ 

/* DHT library
MIT license
written by Adafruit Industries
*/

#include "asf.h"
#include "DHT22.h"

#define DHT22PIN PIN_PB30
#define DHT22PIN_ABSOLUTE PORT_PB30

uint8_t data[5];

////////////////////////////////////////////////////////////////////////////////////////////////////////

static void vResetPort(void)
{ struct port_config pin_conf;
	
	port_get_config_defaults(&pin_conf);
	portENABLE_INTERRUPTS();
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(DHT22PIN, &pin_conf);
	port_pin_set_output_level(DHT22PIN,true);
}

///////////////////////////////////////////////////////////////////////////

static uint16_t expectPulseH(void) 
{
	uint16_t count = 0;

	while ((PORT->Group[1].IN.bit.IN & DHT22PIN_ABSOLUTE))
	 if (count++ == 0x01FF) return 0; // Exceeded timeout, fail.
	 
	return count;
}

///////////////////////////////////////////////////////////////////////////

static uint16_t expectPulseL(void) 
{
	uint16_t count = 0;

	while (!(PORT->Group[1].IN.bit.IN & DHT22PIN_ABSOLUTE))
	 if (count++ == 0x01FF) return 0; // Exceeded timeout, fail.
	 
	return count;
}

///////////////////////////////////////////////////////////////////////////

//static uint16_t expectPulseStart(void) 
//{
	//uint32_t count = 0;
//
	//while ((PORT->Group[1].IN.bit.IN & DHT22PIN_ABSOLUTE))
	 //if (count++ >= 0xFFFFF) return 0; // Exceeded timeout, fail.
	 //
	//return count;
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool read(void) {

	struct port_config pin_conf;
	uint16_t lowCycles,highCycles;
	
	port_get_config_defaults(&pin_conf);
	memset(data,0x00,sizeof(data));

	port_pin_set_output_level(DHT22PIN,false);
	vTaskDelay(TASK_DELAY_MS(2));
	portDISABLE_INTERRUPTS();
	port_pin_set_output_level(DHT22PIN,true);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(DHT22PIN, &pin_conf);
	
	if (!expectPulseH())
	{
		vResetPort();
		return false;
	}	
	
	if (!expectPulseL())
	{
		vResetPort();
		return false;
	}
	
	if (!expectPulseH())
	{
		vResetPort();
		return false;
	}

	for (uint16_t i=0; i<40; i++)
	{
		lowCycles  = expectPulseL();
		highCycles = expectPulseH();
		data[i>>3] <<= 1;
		if (highCycles > lowCycles) data[i>>3] |= 1;
	}
	
	vResetPort();
	
	return (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ? true : false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DHT_Init(void) {

	struct port_config pin_conf;
	
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(DHT22PIN, &pin_conf);
	port_pin_set_output_level(DHT22PIN,true);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//boolean S == Scale.  True == Fahrenheit; False == Celcius
//float DHT_readTemperature() {
	//float f = 0;
//
	//if (read()) {
			//f = data[2] & 0x7F;
			//f *= 256;
			//f += data[3];
			//f *= 0.1;
			//if (data[2] & 0x80) f *= -1;
			////if(S) {
			////	f = convertCtoF(f);
			////}
	//}
	//return f;
//}

//float convertCtoF(float c) {
	//return c * 1.8 + 32;
//}

//float convertFtoC(float f) {
	//return (f - 32) * 0.55555;
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//float DHT_readHumidity() {
	//float f = 0;
	//if (read()) {
			//f = data[0];
			//f *= 256;
			//f += data[1];
			//f *= 0.1;
	//}
	//return f;
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vReadTHAll (float *Tem,float *Hum)
{float f= 0;
 
 if (read()) 
 {
  f = data[2] & 0x7F;
  f *= 256;
  f += data[3];
  f *= 0.1;
  if (data[2] & 0x80) f *= -1;
  //if(S) {
  //	f = convertCtoF(f);
  //}
  *Tem=f;
  f = data[0];
  f *= 256;
  f += data[1];
  f *= 0.1; 
  *Hum=f;
 }
 else
 {
   *Tem=0;
   *Hum=0; 
 }
}

////

////boolean isFahrenheit: True == Fahrenheit; False == Celcius
//float DHT::computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
	//// Using both Rothfusz and Steadman's equations
	//// http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
	//float hi;
//
	//if (!isFahrenheit)
	//temperature = convertCtoF(temperature);
//
	//hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));
//
	//if (hi > 79) {
		//hi = -42.379 +
		//2.04901523 * temperature +
		//10.14333127 * percentHumidity +
		//-0.22475541 * temperature*percentHumidity +
		//-0.00683783 * pow(temperature, 2) +
		//-0.05481717 * pow(percentHumidity, 2) +
		//0.00122874 * pow(temperature, 2) * percentHumidity +
		//0.00085282 * temperature*pow(percentHumidity, 2) +
		//-0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);
//
		//if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
		//hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);
//
		//else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
		//hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
	//}
//
	//return isFahrenheit ? hi : convertFtoC(hi);
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

