#include "sound.h"

#define FREQUENCY_MODE_ABORTED_WAYPOINT 800
#define FREQUENCY_MODE_ALARM 200

const char soundFinished[] PROGMEM = "! L16 V8 cdefgab>cbagfedc";
const char soundOk[] PROGMEM = "v10>>g16>>>c16";
const char soundBoot[] PROGMEM = "v10>>a16>>e16>>a16";

void Sound::finished()
{
    buzzer.playFromProgramSpace(soundFinished);
    delay(2000);  
}

void Sound::ok()
{
    buzzer.playFromProgramSpace(soundOk);
    delay(500);  
}

void Sound::boot()
{
    buzzer.playFromProgramSpace(soundBoot);
    delay(500);  
}

void Sound::pip()
{
    playFrequency(FREQUENCY_MODE_ALARM, 100, FREQUENCY_VOLUME);
}

void Sound::alarm(bool wait, uint8_t repetitions)
{
    for (uint8_t i = 0; i < repetitions; i++)
    {
        playFrequency(FREQUENCY_MODE_ALARM, FREQUENCY_DURATION, FREQUENCY_VOLUME);
        if (wait)
        {
            delay(FREQUENCY_DURATION);
        }
    }
}

void Sound::abortedWaypoint(uint8_t repetitions)
{
    for (uint8_t i = 0; i < repetitions; i++)
    {
        playFrequency(FREQUENCY_MODE_ABORTED_WAYPOINT, FREQUENCY_DURATION, FREQUENCY_VOLUME);
        delay(FREQUENCY_DURATION * 2);
    }
}

void Sound::abortedWaypointDueToObstacle()
{
    abortedWaypoint(4);
}

void Sound::abortedWaypointDueToDistanceIncrease()
{
    abortedWaypoint(3);
}

void Sound::abortedWaypointDueToStall()
{
    abortedWaypoint(2);
}

void Sound::failedObstacleAvoidance()
{
    alarm(true, 5);
}

void Sound::playFrequency(int mode, int duration, int volume)
{
    buzzer.playFrequency(mode, duration, volume);
}

