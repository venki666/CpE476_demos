#ifdef __DEMOS__

/****************************************************************************************************************
 * Demos 
 ****************************************************************************************************************/

double waypointsSquare[][2] = {
    {100.0, 0.0},
    {100.0, 100.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

double waypointsLine[][2] = {
    {100.0, 0.0}
};

double waypointsRectangle[][2] = {
    {-50.0, 50.0},
    {50.0, 50.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

double waypointsStar[][2] = {
    {-40.0, 90.0},
    {-80.0, 0.0},
    {40.0, 60.0},
    {-100.0, 60.0},
    {0.0, 0.0}
};

double waypointsTriangles[][2] = {
    {50.0, 50.0},
    {100.0, 0.0},
    {100.0, 100.0},
    {50.0, 50.0},
    {0.0, 100.0},
    {0.0, 0.0}
};

#define NUM_WAYPOINTS 6
double (*waypoints)[2] = waypointsTriangles;

void preprogrammedWaypoints()
{
    BotCmdCtx cmdCtx;

    cmdCtx.maxVelocity = MAX_VELOCITY;
    
#ifdef __DEBUG__                      
    snprintf_P(report, sizeof(report), PSTR("************************ START ************************"));    
    Serial.println(report);
#endif
    
    resetToOrigin();
    for (int i = 0; i < NUM_WAYPOINTS; i++)
    {
        goToWaypoint(waypoints[i][0], waypoints[i][1], cmdCtx);
    }

#ifdef __DEBUG__                    
    snprintf_P(report, sizeof(report), PSTR("************************ END ************************"));    
    Serial.println(report);      
#endif

    buzzer.playFromProgramSpace(soundFinished);
}

void gyroBasedOrientation() 
{
    resetToOrigin();

    correctHeadingWithPivotTurnGyro(-1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(-1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57, PIVOT_TURN_SPEED, false);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnGyro(1.57, PIVOT_TURN_SPEED, false);  
}

#ifdef ENABLE_MAGNETOMETER
void magnetometerBasedOrientation() 
{
    resetToOrigin();

    correctHeadingWithPivotTurnMagnetometer(-1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(-1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57, PIVOT_TURN_SPEED);
    delay(100);
    resetDistancesForNewWaypoint();
    correctHeadingWithPivotTurnMagnetometer(1.57, PIVOT_TURN_SPEED);  
}
#endif

#endif // __DEMOS__
