#include <Romi32U4.h>
#include <Odometer.h>

const float tickDist = .152505;
const float track = 142.5;

Romi32U4Encoders encoders;
Odometer odometer(tickDist, track);

Romi32U4Motors motors;
Romi32U4ButtonA btnA;
Romi32U4ButtonB btnB;
Romi32U4ButtonC btnC;

void setup() {
  Serial.begin(9600);
  //encoders.flipDirection(false, true);
}

void loop() {
  // Get encoder counts
  int left = encoders.getCountsLeft();
  int right = encoders.getCountsRight();

  // Update odometer
  odometer.update(left, right);

  Serial.print(odometer.getX());
  Serial.print("\t");
  Serial.print(odometer.getY());
  Serial.print("\t");
  Serial.print(odometer.getPhi()*100);
  Serial.print("\t");
  Serial.print(odometer.getSpeed());
  Serial.print("\t");
  Serial.println(odometer.getOmega()*100);

  if (btnA.isPressed())
  {
    // Turn left
    motors.setSpeeds(-50, 50);
  }
  else if (btnB.isPressed())
  {
    // Turn right
    motors.setSpeeds(50, -50);
  }
  else if (btnC.isPressed())
  {
    // Go straight fwd
    motors.setSpeeds(100, 100);
  }
  else
  {
    motors.setSpeeds(0, 0);
  }
  delay(10);
}
