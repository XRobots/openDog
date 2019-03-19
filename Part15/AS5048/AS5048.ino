#include <SPI.h>	// Throws an error in Windows
#include <AS5048A.h>
#include <Streaming.h>
/*
 * To use the streaming library:
 * http://arduiniana.org/libraries/streaming/
 *
 */

/**
 * Basic angle procedure:
 *
 * - Obtain register readings (zero position and current position)
 * - Transform to angles with read2angle() 
 * - Calculate the difference between them
 * - normalize() the result
 *
 * NOTE:
 * To see the angles between 0° and 360° comment the following
 * #define, instead, to see the angles between -180° and 180°,
 * uncomment it.
 *
 */

//#define ANGLE_MODE_1	// Between -180° and 180°

AS5048A angleSensor(10);
uint16_t zero_position;
uint16_t zero_position_map;

float inline read2angle(uint16_t angle) {
	/*
	 * 14 bits = 2^(14) - 1 = 16.383
	 *
	 * https://www.arduino.cc/en/Reference/Map
	 *
	 */
	return angle * ((float)360 / 16383);
};

float normalize(float angle) 
{
	// http://stackoverflow.com/a/11498248/3167294
#ifdef ANGLE_MODE_1
	angle += 180;
#endif
	angle = fmod(angle, 360);
	if (angle < 0) {
		angle += 360;
	}
#ifdef ANGLE_MODE_1
	angle -= 180;
#endif
	return angle;
}

void setup()
{
	Serial.begin(115200);
	angleSensor.init();

	zero_position = angleSensor.getRawRotation();
	zero_position_map = read2angle(zero_position);

	Serial << "> Zero: " << zero_position
			<< "\t" << zero_position_map << endl;
}

void loop()
{
	delay(20);

	uint16_t current_angle = angleSensor.getRawRotation();
	float current_angle_map = read2angle(current_angle);

	float angle = current_angle_map - zero_position_map;
	angle = normalize(angle);

  uint16_t tenbit = map(current_angle, 0, 16383, 0, 1023);

	Serial << current_angle << "\t"
      << tenbit << "\t"  
			<< angle 
			<< endl << endl;

  
/*	
	Serial << rel_angle
			<< "\t" << _BIN(rel_angle) 
			<< "\t" << _HEX(rel_angle) << endl;

*/

	if (angleSensor.error()) {
		Serial << "ERROR: " << angleSensor.getErrors() << endl;
	}
}
