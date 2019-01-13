#include "custom_util.h"

const double DT = 0.016667;

const double THROTTLE_ACCEL = 1600;
const double BREAK_ACCEL = 3500;
const double BOOST_ACCEL = 991.6667;
const double BOOST_CONSUMPTION_RATE = 0.333;

const double THROTTLE_MAX_SPEED = 1400;
const double MAX_CAR_SPEED = 2300;


double turning_speed(double radius) {
    return 10.219 * radius - 1.75404E-2 * pow(radius, 2) + 1.49406E-5 * pow(radius, 3) - 4.486542E-9 * pow(radius, 4) - 1156.05;
}



double throttle_acc(double v, double throttle = 1)
{
    if (Sign(v) == Sign(throttle)) {
        return (-THROTTLE_ACCEL / THROTTLE_MAX_SPEED * std::min(abs(v), THROTTLE_MAX_SPEED) + THROTTLE_ACCEL) * throttle;
    }
    else {
        return BREAK_ACCEL * Sign(throttle);
    }
}



double max_travel_distance(double max_time, double v_0, double initial_boost)
{
    double time = 0;
    double distance = 0;
    double velocity = v_0;
    double boost = initial_boost;
    double acceleration = 0;

    while (time < max_time)
    {
        acceleration = throttle_acc(velocity) + (boost > 1) * BOOST_ACCEL;
        velocity = std::min(velocity + acceleration * DT, MAX_CAR_SPEED);
        distance = distance + velocity * DT + 0.5 * acceleration * DT * DT;
        boost -= BOOST_CONSUMPTION_RATE * 100 * DT;
        time += DT;
        if (time > 10) {
            break;
        }
    }

    return distance;
}

double min_travel_time(double max_distance, double v_0, double initial_boost)
{
    double time = 0;
    double distance = 0;
    double velocity = v_0;
    double boost = initial_boost;
    double acceleration = 0;

    while (distance < max_distance)
    {
        acceleration = throttle_acc(velocity) + (boost > 1) * BOOST_ACCEL;
        velocity = std::min(velocity + acceleration * DT, MAX_CAR_SPEED);
        distance = distance + velocity * DT + 0.5 * acceleration * DT * DT;
        boost -= BOOST_CONSUMPTION_RATE * 100 * DT;
        time += DT;
        if (time > 10) {
            break;
        }
    }

    return time;
}
