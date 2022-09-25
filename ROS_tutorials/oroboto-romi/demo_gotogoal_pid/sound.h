#ifndef __SOUND_H__
#define __SOUND_H__

#include <Romi32U4.h>

#define FREQUENCY_DURATION 300
#define FREQUENCY_VOLUME 9

class Sound
{
  public:
    void finished();
    void ok();
    void boot();
    void pip();
    void alarm(bool wait, uint8_t repetitions = 1);
    void failedObstacleAvoidance();
    void abortedWaypointDueToDistanceIncrease();
    void abortedWaypointDueToStall();
    void abortedWaypointDueToObstacle();

  private:
    void playFrequency(int mode, int duration, int volume);
    void abortedWaypoint(uint8_t repetitions);
    
    Romi32U4Buzzer buzzer;
};

#endif // __SOUND_H__
