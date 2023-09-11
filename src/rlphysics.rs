use crate::custom_util::*;
use pyo3::prelude::*;

const DT: f64 = 0.016667;

const THROTTLE_ACCEL: f64 = 1600.0;
const BREAK_ACCEL: f64 = 3500.0;
const BOOST_ACCEL: f64 = 991.6667;
const BOOST_CONSUMPTION_RATE: f64 = 0.333;

const THROTTLE_MAX_SPEED: f64 = 1400.0;
const MAX_CAR_SPEED: f64 = 2300.0;

// pub fn turning_speed(radius: f64) -> f64 {
//     return 10.219 * radius - 1.75404E-2 * radius.powf(2.0) + 1.49406E-5 * radius.powf(3.0)
//         - 4.486542E-9 * radius.powf(4.0)
//         - 1156.05;
// }

pub fn throttle_acc(v: f64, throttle: f64) -> f64 {
    if custom_sign(v) == custom_sign(throttle) {
        return (-THROTTLE_ACCEL / THROTTLE_MAX_SPEED * v.abs().min(THROTTLE_MAX_SPEED)
            + THROTTLE_ACCEL)
            * throttle;
    } else {
        return BREAK_ACCEL * custom_sign(throttle);
    }
}

#[pyfunction]
pub fn max_travel_distance(max_time: f64, v_0: f64, initial_boost: f64) -> f64 {
    let mut time: f64 = 0.0;
    let mut distance: f64 = 0.0;
    let mut velocity = v_0;
    let mut boost = initial_boost;
    let mut acceleration: f64;

    while time < max_time {
        acceleration = throttle_acc(velocity, 1.0) + ((boost > 1.0) as u8 as f64) * BOOST_ACCEL;
        velocity = (velocity + acceleration * DT).min(MAX_CAR_SPEED);
        distance = distance + velocity * DT + 0.5 * acceleration * DT * DT;
        boost -= BOOST_CONSUMPTION_RATE * 100.0 * DT;
        time += DT;
        if time > 10.0 {
            break;
        }
    }

    return distance;
}

#[pyfunction]
pub fn min_travel_time(max_distance: f64, v_0: f64, initial_boost: f64) -> f64 {
    let mut time: f64 = 0.0;
    let mut distance: f64 = 0.0;
    let mut velocity = v_0;
    let mut boost = initial_boost;
    let mut acceleration: f64;

    while distance < max_distance {
        acceleration = throttle_acc(velocity, 1.0) + ((boost > 1.0) as u8 as f64) * BOOST_ACCEL;
        velocity = (velocity + acceleration * DT).min(MAX_CAR_SPEED);
        distance = distance + velocity * DT + 0.5 * acceleration * DT * DT;
        boost -= BOOST_CONSUMPTION_RATE * 100.0 * DT;
        time += DT;
        if time > 10.0 {
            break;
        }
    }

    return time;
}
