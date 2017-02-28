/*
 * DHT22.h
 *
 * Created: 9/22/2016 09:53:27
 *  Author: ROBERTOC
 */ 


#ifndef DHT22_H_
#define DHT22_H_

void DHT_Init(void);
//float DHT_readTemperature(void);
//float DHT_readHumidity(void);
void vReadTHAll (float *,float *);

#endif /* DHT22_H_ */