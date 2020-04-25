#ifndef MOTION_CONTROL_FUNCTIONS_HPP
#define MOTION_CONTROL_FUNCTIONS_HPP

#include <ros/ros.h>
#include <math.h>
#include <algorithm>

//
// Macro definitions
//
#define POS_UPDATE_PERIOD .025
#define POS_UPDATE_RATE (int)(1 / POS_UPDATE_PERIOD)
#define COUNTS_PER_REV 800
#define ENCODER_WHEEL_DIAMETER 2.875
#define PI 3.1415926
#define BUFFER_SIZE 50000
#define MAX_VELO_IN_SEC 42
#define THRESHOLD_VEL_IN_SEC 2.9
#define STALL_VEL_IN_SEC THRESHOLD_VEL_IN_SEC - .1
#define TRAJ_START_VEL_IN_SEC 0.0
#define TRIA_START_VEL_IN_SEC 0.0
#define MAX_ACCEL_IN_SEC_SEC 16
#define IN_PER_METER 1.0
#define MAX_STALL_COUNT 80
#define DEFAULT_RFID_VALUE 0
#define COUNTER_UPDATE_PERIOD .01
#define FIND_HOME_VELOCITY 8.
#define FIND_CHARGER_VELOCITY 4.
#define STEP_VELOCITY_INCH_S 15
#define STEP_WAIT_TIME 0.2
#define FORWARD 1
#define BACKWARD -1
#define MAX_TRACK_LENGTH 5000.
#define MAGNET_LOCATION_IN 0.

static int epochs = 0;
//
// default max velocity in inches per second
//
static float maxVelocityIn = (float)MAX_VELO_IN_SEC;
//
// default max acceleration inches per second per second
//
static float maxAccelerationIn = (float)MAX_ACCEL_IN_SEC_SEC;
//
// metric equivalents
//
static float maxVelocityM = maxVelocityIn / (float)IN_PER_METER;
static float maxAccelerationM = maxAccelerationIn / (float)IN_PER_METER;
//
// index within the trajectory buffer
//
static int trajectoryIndex = 0;
//
// position and PID parameters
//
static float current_velocity_error = 0.0;
static float previous_velocity_error = 0.0;
static float change_velocity_error = 0.0;
static float cummulative_velocity_error = 0.0;

static float current_position_error = 0.0;
static float previous_position_error = 0.0;
static float change_position_error = 0.0;
static float cummulative_position_error = 0.0;

static const float move_speed_ratio = 1.0;
static const float find_charger_ratio = 0.12;

// Used to keep track of the PID control variane
static std::vector<float> pidHistory;

// Keeps track of the variance of the PID control output
static float pidVariance;

// Time window to save the PID outputs
#define PID_HISTORY_SIZE 30

// The variance required to turn off motion control
#define PID_VAR_THRESHOLD 0.01

//
// assume:
//      1. constant acceleration, acceleration is known
//      2. total distance is given as an input
//      3. peak velocity and time are not known
//      4. peak velocity cannot exceed max velocity
//      5. distance is the area under the velocity vs time curve
//
// parameter speedReduction allows moving at slower speed for seek
// and findcharger.
//
// return trajectory length
//
int computeTrajectory(float *cmdVelBuffer,
                      float *desPosBuffer,
                      float distance,
                      float speedReduction,
                      float currentPosition)
{

    // Compute the maximum velocity and acceleration
    float maxVelocity = maxVelocityM * speedReduction;
    float maxAcceleration = maxAccelerationM;

    // Compute the maximum possible velocity for 1/2 the distance of the move
    float possibleVelocity = std::min((float)sqrt(2 * maxAcceleration * ((std::abs(distance / 2.)))), maxVelocity);

    // Time to ramp up = Vmax / Amax
    float rampTime = possibleVelocity / maxAcceleration;

    // Distance covered during ramp up = 0.5 * Amax * rampTime^2
    float rampDistance = 0.5 * maxAcceleration * rampTime * rampTime;

    // Compute the distance required for the flat part of the trajectory
    float flatDistance = std::max((float)0., std::abs(distance) - 2 * rampDistance);

    // Compute the time required to complete the flat move. Since the flat move only happens
    // when the robot can reach peak velocity in the move, we use the max velocity here
    float flatTime = flatDistance / maxVelocity;

    // Compute the total amount of time required to make the move from the ramp up/down
    float totalTime = 2 * rampTime;

    // If the flat part of the trajectory exists, add in its time as well
    if (flatDistance > 0.)
        totalTime += flatTime;

    float sign = (distance > 0) ? 1. : -1.;

    // Compute the total amount of epochs in the trajectory
    epochs = (int)(totalTime / POS_UPDATE_PERIOD);

    // Create the trajectory profile
    for (int i = 0; i < epochs; i++)
    {
        // Compute the current time this iteration represents
        float t = (float)i * POS_UPDATE_PERIOD;

        // Ramping up
        if (t <= rampTime)
        {
            // s = 0.5 Amax * t^2
            desPosBuffer[i] = sign * 0.5 * maxAcceleration * t * t + currentPosition;
        }
        // Flat velocity
        else if (flatDistance > 0. && t <= rampTime + flatTime)
        {
            // Compute the start time of this portion of the trajectory
            float legTime = (t - rampTime);

            // s = s0 + t*Vmax
            desPosBuffer[i] = sign * (rampDistance + legTime * maxVelocity) + currentPosition;
        }
        // Ramping down
        else
        {
            // Compute the start time of this portion of the trajectory
            float legTime = t - rampTime - flatTime;

            // Compute the total distance moved so far
            float movedDistance = rampDistance + std::max((float)0., flatDistance);

            // s = s0 + v0 * t + 0.5 -Amax * t^2
            desPosBuffer[i] = sign * (movedDistance + possibleVelocity * legTime - 0.5 * maxAcceleration * legTime * legTime) + currentPosition;
        }
    }

    return epochs;
}

//
// compute the PID value using the equation:
//      PID = kp*err + kd*err_rate + ki*cum_err
//
float computePIDVel(float currentVel, float desiredVel, float dt)
{
    float pid;
    float kp = 0.042;
    float kd = 0.00005;
    float ki = 0.03;
    //
    // compute the velocity errors
    //
    // velocity error = desired velocity - actual velocity
    // integral = integral + error, need to initialize to zero
    // diff error = (current error - previous error) / time slice
    // PID = delta pwm = kp * error + kd * diff error + ki * integral
    //
    current_velocity_error = desiredVel - currentVel;
    change_velocity_error = (current_velocity_error - previous_velocity_error) / dt;
    cummulative_velocity_error += current_velocity_error * dt;
    pid = (kp * current_velocity_error) +
          (kd * change_velocity_error) +
          (ki * cummulative_velocity_error);
    previous_velocity_error = current_velocity_error;

    return pid;
}

//
// compute the PID value using the equation:
//      PID = kp*err + kd*err_rate + ki*cum_err
//
float computePIDPos(float currentPos, float desiredPos, bool steppingMode, float dt)
{
    float pid, kp, ki, kd;

    // Set the gains ba
    if(!steppingMode)
    {
        kp = 3.5;
        kd = 0.0;
        ki = 0.0;
    }
    else
    {
        kp = 9.;
        kd = 0.0;
        ki = 0.0;
    }

    //
    // compute the position errors
    //
    // position error = desired position - actual position
    // integral = integral + error, need to initialize to zero
    // diff error = (current error - previous error) / time slice
    // PID = delta pwm = kp * error + kd * diff error + ki * integral
    //
    current_position_error = desiredPos - currentPos;
    change_position_error = (current_position_error - previous_position_error) / dt;
    cummulative_position_error += current_position_error * dt;
    pid = (kp * current_position_error) +
          (kd * change_position_error) +
          (ki * cummulative_position_error);
    previous_position_error = current_position_error;

    return pid;
}

//
// return true when trajectory execution is complete, false otherwise
//
bool executeTrajectory(float *cmdVelBuffer, float *desPosBuffer, float distance,
                       float *cmd_vel, float currVel, float currPos, float dt, bool steppingMode=false)
{

    float desired_cmd_pos = 0.0;

    // Check and see if we've accumulated enough data to compute the variance,
    // and if we've gone below the variance threshold
    if (pidHistory.size() == PID_HISTORY_SIZE && pidVariance < PID_VAR_THRESHOLD)
    {
        trajectoryIndex = 0;

        current_velocity_error = 0.0;
        previous_velocity_error = 0.0;
        change_velocity_error = 0.0;
        cummulative_velocity_error = 0.0;

        current_position_error = 0.0;
        previous_position_error = 0.0;
        change_position_error = 0.0;
        cummulative_position_error = 0.0;

        pidHistory.clear();

        return false;
    }
    // If we've reached the end of the computed trajectory, start collecting
    // data on the PID control output so we can compute its variance. The goal position
    // is set to the end of the trajectory so the robot can converge on the final position
    else if (trajectoryIndex == epochs)
    {
        float mean = 0;
        float pos_pid = computePIDPos(currPos, desPosBuffer[trajectoryIndex - 1], steppingMode, dt);
        *cmd_vel = (*cmd_vel) + pos_pid;

        // Add the latest PID control value to the end of the history
        if (pidHistory.size() == PID_HISTORY_SIZE)
            pidHistory.erase(pidHistory.begin());

        pidHistory.push_back(pos_pid);

        // Pop off the top of the history if it is full
        if (pidHistory.size() == PID_HISTORY_SIZE)
        {
            // Compute the mean
            for (int i = 0; i < PID_HISTORY_SIZE; i++)
                mean += pidHistory[i];
 
            mean /= (float)PID_HISTORY_SIZE;

            pidVariance = 0;

            // Compute the variance
            for (int i = 0; i < PID_HISTORY_SIZE; i++)
                pidVariance += (pidHistory[i] - mean) * (pidHistory[i] - mean);

            pidVariance /= (float)(PID_HISTORY_SIZE - 1);
        }
    }
    else
    {
        desired_cmd_pos = desPosBuffer[trajectoryIndex];

        float pos_pid = computePIDPos(currPos, desired_cmd_pos, steppingMode, dt);
        *cmd_vel = (*cmd_vel) + pos_pid;

        //
        // increment the index value for the next iteration
        //
        ++trajectoryIndex;
    }

    return true;
}

#endif // MOTION_CONTROL_FUNCTIONS_HPP
