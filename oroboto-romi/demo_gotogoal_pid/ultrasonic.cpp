#include <Romi32U4.h>
#include "ultrasonic.h"

#define VCC 5.0
#define VOLTS_PER_CM_DIVISOR 1300.48              // 512.0 * 2.54 (from datasheet)
#define VOLTS_PER_CM VCC / VOLTS_PER_CM_DIVISOR   // 0.003844734251969
#define ADC_LSB_PER_VOLT 1023 / VCC               // 1023 / 5.0

UltrasonicTransducer::UltrasonicTransducer()
{
    pinMode(A4, INPUT); // use the ADC on A4 for ultrasonics  
}


/**
 * Get the current distance measured from the ultrasonic sensor. Returns a value in centimeters.
 */
double UltrasonicTransducer::getSonarRangedDistance()
{
    double distance = 0;

    int adc = analogRead(A4);
    double mv = ((double)adc / (ADC_LSB_PER_VOLT)) * 1000.0;

    distance = mv / (VOLTS_PER_CM * 1000.0);

    return distance;
}
