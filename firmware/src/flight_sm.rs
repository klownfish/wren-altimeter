#![allow(dead_code)]

use core::fmt::{self, Display};

#[allow(unused_imports)]
#[cfg(target_os = "none")]
use defmt::{debug, error, info, warn};
#[allow(unused_imports)]
#[cfg(not(target_os = "none"))]
use log::{debug, error, info, warn};
#[cfg(target_os = "none")]
use nalgebra::{ComplexField, RealField};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use embassy_time::Instant;

use crate::kalman::{self, Kalman};

#[repr(u8)]
#[cfg_attr(target_os = "none", derive(defmt::Format))]
#[derive(PartialEq, Clone, Copy, Debug)]
pub enum FlightState {
    Idle = 0,
    MaybeLaunched = 1,
    Boost = 2,
    Coast = 3,
    Descent = 4,
}

impl Display for FlightState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlightState::Idle => write!(f, "Idle"),
            FlightState::MaybeLaunched => write!(f, "Maybe Launched"),
            FlightState::Boost => write!(f, "Boost"),
            FlightState::Coast => write!(f, "Coast"),
            FlightState::Descent => write!(f, "Descent"),
        }
    }
}

pub struct FlightSM {
    last_update: Instant,
    state: FlightState,
    kalman: kalman::Kalman<3>,
    imu_orientation: UnitQuaternion<f32>,
    last_state_transition: Instant,
    ground_level: f32,
    last_altitude: f32,
    descent_counter: u16,
    raw_acceleration: f32,
}

fn pressure_to_altitude(pressure: f64) -> f32 {
    // ISA constants
    const P0: f64 = 1013.250; // Sea-level pressure in hPa
    const T0: f64 = 288.15; // Sea-level temperature in K
    const L: f64 = 0.0065; // Temperature lapse rate in K/m
    const G: f64 = 9.81; // Gravitational acceleration constant in m/s^2
    const R: f64 = 287.05; // Specific gas constant for dry air in J/(kg·K)

    let altitude = (T0 / L) * (1.0 - (pressure / P0).powf(R * L / G));
    altitude as f32
}

impl FlightSM {
    // measurement noise
    const SIGMA_MEASUREMENT_ACCELERATION_ASCENT: f32 = 0.001;
    const SIGMA_MEASUREMENT_ALTITUDE_ASCENT: f32 = 1.0;
    const SIGMA_MEASUREMENT_ALTITUDE_DESCENT: f32 = 30.0;

    // process noise
    const SIGMA_PROCESS_ACCELLERATION: f32 = 1.0;
    const SIGMA_PROCESS_VELOCITY: f32 = 0.01;
    const SIGMA_PROCESS_ALTITUDE: f32 = 0.001;

    const BAROMETER_LP_DESCENT: f32 = 0.2;

    const IMU_ORIENTATION_LP: f32 = 0.1;

    const GRAVITY: f32 = 9.82;
    const GRAVITY_LOWER_THRESHOLD: f32 = 9.5;
    const GRAVITY_UPPER_THRESHOLD: f32 = 10.1;

    const SONIC_SHOCK_LOWER_THRESHOLD: f32 = 310.0;
    const SONIC_SHOCK_UPPER_THRESHOLD: f32 = 370.0;

    const IDLE_ACCELERATION_THRESHOLD: f32 = 15.0;
    const MAYBE_LAUNCHED_ALTITUDE_THRESHOLD: f32 = 5.0;
    const MAYBE_LAUNCHED_TIME_THRESHOLD: f32 = 3.0;

    const DESCENT_HOLDOFF: f32 = 10.0;

    pub fn new() -> Self {
        Self {
            kalman: Kalman::<3>::new(),
            state: FlightState::Idle,
            last_update: Instant::now(),
            imu_orientation: UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            last_state_transition: Instant::now(),
            ground_level: 0.0,
            last_altitude: 0.0,
            descent_counter: 0,
            raw_acceleration: 0.0,
        }
    }

    pub fn update(&mut self, pressure: f64, acceleration: Vector3<f32>) {
        let now = Instant::now();
        let dt: f32 = now.duration_since(self.last_update).as_millis() as f32 / 1000.0;
        self.last_update = now;

        let new_altitude: f32 = pressure_to_altitude(pressure);
        let altitude: f32;

        if self.state == FlightState::Descent {
            altitude =
                new_altitude * Self::BAROMETER_LP_DESCENT + self.last_altitude * (1.0 - Self::BAROMETER_LP_DESCENT);
        } else {
            altitude = new_altitude;
        }
        self.last_altitude = altitude;
        // accel stuff
        let accel_norm = acceleration.norm();
        self.raw_acceleration = accel_norm;
        if accel_norm < Self::GRAVITY_UPPER_THRESHOLD && accel_norm > Self::GRAVITY_LOWER_THRESHOLD {
            let new_imu_orientation = Self::calc_imu_orientation_correction(acceleration);
            self.imu_orientation = self
                .imu_orientation
                .slerp(&new_imu_orientation, Self::IMU_ORIENTATION_LP);
            // wtf why does cargo fmt do it like this. I am literally going to vomit
        }
        let rotated_acceleration: Vector3<f32> = self.imu_orientation * acceleration;
        let vertical_acceleration: f32 = rotated_acceleration.z - Self::GRAVITY; // TODO use norm but find sign
                                                                                 // let vertical_acceleration: f32 = accel_norm - Self::GRAVITY;

        // prepare kalman process
        #[rustfmt::skip]
        let a = kalman::AMatrix::<3>::new(
            1.0, dt, dt * dt / 2.0,
            0.0, 1.0, dt,
             0.0, 0.0, 1.0
        );

        #[rustfmt::skip]
        let q = kalman::QMatrix::<3>::new(
            Self::SIGMA_PROCESS_ALTITUDE, 0.0, 0.0,
            0.0, Self::SIGMA_PROCESS_VELOCITY, 0.0,
            0.0, 0.0, Self::SIGMA_PROCESS_ACCELLERATION,
        );

        self.kalman.set_process(&q, &a);

        // prepare kalman measurement

        // descent phase -> only baro
        // close to sonic -> only accel
        // otherwise both
        if self.state == FlightState::Descent {
            let h = kalman::HMatrix::<3, 1>::new(1.0, 0.0, 0.0);
            let z = kalman::ZMatrix::<3, 1>::new(altitude);
            let r = kalman::RMatrix::<3, 1>::new(Self::SIGMA_MEASUREMENT_ALTITUDE_DESCENT);
            self.kalman.insert_measurement(&z, &r, &h);
        } else if Self::SONIC_SHOCK_LOWER_THRESHOLD < self.get_vertical_velocity()
            && self.get_vertical_velocity() < Self::SONIC_SHOCK_UPPER_THRESHOLD
        {
            let h = kalman::HMatrix::<3, 1>::new(0.0, 0.0, 1.0);
            let z = kalman::ZMatrix::<3, 1>::new(vertical_acceleration);
            let r = kalman::RMatrix::<3, 1>::new(Self::SIGMA_MEASUREMENT_ACCELERATION_ASCENT);
            self.kalman.insert_measurement(&z, &r, &h);
        } else {
            #[rustfmt::skip]
            let h = kalman::HMatrix::<3, 2>::new(
                1.0, 0.0, 0.0,
                0.0, 0.0, 1.0
            );
            let z = kalman::ZMatrix::<3, 2>::new(altitude, vertical_acceleration);
            let r = kalman::RMatrix::<3, 2>::new(
                Self::SIGMA_MEASUREMENT_ALTITUDE_ASCENT,
                0.0,
                0.0,
                Self::SIGMA_MEASUREMENT_ACCELERATION_ASCENT,
            );
            self.kalman.insert_measurement(&z, &r, &h);
        }

        // let the kalman filter stabilize
        if Instant::now().as_millis() < 10000 {
            self.ground_level = self.get_absolute_altitude();
            return;
        }

        // evaluate state
        let old_state = self.state.clone();
        match self.state {
            FlightState::Idle => {
                if self.get_vertical_acceleration() > Self::IDLE_ACCELERATION_THRESHOLD {
                    self.state = FlightState::MaybeLaunched;
                }
                self.ground_level = self.get_absolute_altitude();
            }

            FlightState::MaybeLaunched => {
                let in_state_for: f32 =
                    Instant::now().duration_since(self.last_state_transition).as_millis() as f32 / 1000.0;
                if self.get_relative_altitude() > Self::MAYBE_LAUNCHED_ALTITUDE_THRESHOLD {
                    self.state = FlightState::Boost;
                } else if in_state_for > Self::MAYBE_LAUNCHED_TIME_THRESHOLD {
                    self.state = FlightState::Idle;
                }
            }

            FlightState::Boost => {
                if self.get_vertical_acceleration() < 0.0 {
                    self.state = FlightState::Coast;
                }
            }

            FlightState::Coast => {
                if self.get_vertical_velocity() < 0.0 {
                    self.state = FlightState::Descent;
                    self.descent_counter = 0;
                }
            }

            FlightState::Descent => {
                let in_state_for: f32 =
                    Instant::now().duration_since(self.last_state_transition).as_millis() as f32 / 1000.0;
                if in_state_for > Self::DESCENT_HOLDOFF && self.get_vertical_velocity().abs() < 1.0 {
                    self.descent_counter += 1;
                    if self.descent_counter > 250 {
                        self.state = FlightState::Idle;
                    }
                } else {
                    self.descent_counter = 0;
                }
            }
        }

        if old_state != self.state {
            self.last_state_transition = Instant::now();
        }
    }

    pub fn get_state(&self) -> FlightState {
        self.state
    }

    pub fn get_absolute_altitude(&self) -> f32 {
        self.kalman.get_state()[0]
    }

    pub fn get_relative_altitude(&self) -> f32 {
        self.kalman.get_state()[0] - self.ground_level
    }

    pub fn get_vertical_velocity(&self) -> f32 {
        self.kalman.get_state()[1]
    }

    pub fn get_vertical_acceleration(&self) -> f32 {
        self.kalman.get_state()[2]
    }

    pub fn get_raw_acceleration(&self) -> f32 {
        self.raw_acceleration
    }

    fn calc_imu_orientation_correction(measured: Vector3<f32>) -> UnitQuaternion<f32> {
        // Normalize the measured vector
        let mut measured_vector = measured.normalize();
        // Calculate rotation around x-axis
        let x_rot_angle = measured_vector.y.atan2(measured_vector.z) / 2.0;
        let x_rot_quat =
            UnitQuaternion::from_quaternion(Quaternion::new(x_rot_angle.cos(), x_rot_angle.sin(), 0.0, 0.0));

        // Apply x-axis rotation to the measured vector and normalize
        measured_vector = x_rot_quat * measured_vector;
        measured_vector = measured_vector.normalize();

        // Calculate rotation around y-axis
        let y_rot_angle = -measured_vector.x.atan2(measured_vector.z) / 2.0;
        let y_rot_quat =
            UnitQuaternion::from_quaternion(Quaternion::new(y_rot_angle.cos(), 0.0, y_rot_angle.sin(), 0.0));

        // Combine the rotations to get the final orientation
        let result = y_rot_quat * x_rot_quat;
        result
    }
}
