#ifndef SERVOS_H
#define SERVOS_H


#define N_SERVOS 12

// This delay in milliseconds determines the pause
// between each one degree step the servo travels.  Increasing
// this number will make the servo sweep more slowly.
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
int stepDelay [N_SERVOS] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 }; // ms

// Pins
byte servoPins [N_SERVOS] = { 22, 23, 24, 25, 26, 27, 40, 41, 42, 43, 44, 45 };

// Initial Position
byte servoInitPosition [N_SERVOS] = { 135, 0, 0, 0, 60, 90, 45, 180, 0, 180, 60, 90 }; // [0, 180] degrees


class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    int getServoPosition();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif
