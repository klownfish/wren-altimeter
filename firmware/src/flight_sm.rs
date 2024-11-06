#![allow(dead_code)]

use nalgebra::{ComplexField, Quaternion, RealField, UnitQuaternion, Vector3};
use embassy_time::Instant;

use crate::kalman::{self, Kalman};

#[derive(PartialEq, Clone)]
pub enum FlightState {
    Idle,
    MaybeLaunched,
    Boost,
    Coast,
    Descent
}

pub struct FlightSM {
    last_update: Instant,
    state: FlightState,
    kalman: kalman::Kalman<3>,
    imu_orientation: UnitQuaternion<f32>,
    last_state_transition: Instant
}

impl FlightSM {
    // measurement noise
    const SIGMA_MEASUREMENT_ACCEL: f32 = 0.1;
    const SIGMA_MEASUREMENT_ALTITUDE: f32 = 1.0;

    // AKA process noise
    // need two values, ascent -> fast changes
    // descent -> slow changes
    const SIGMA_PROCESS_ACCEL_ASCENT: f32 = 0.1;
    const SIGMA_PROCESS_ACCEL_DESCENT: f32 = 0.1;

    const GRAVITY: f32 = 9.82;
    const GRAVITY_LOWER_THRESHOLD: f32 = 9.4;
    const GRAVITY_UPPER_THRESHOLD: f32 = 10.2;

    const IDLE_ACCELERATION_THRESHOLD: f32 = 20.0;
    const MAYBE_LAUNCHED_ALTITUDE_THRESHOLD: f32 = 15.0;
    const MAYBE_LAUNCHED_TIME_THRESHOLD: f32 = 1.0;

    const DESCENT_HOLDOFF: f32 = 10.0;

    pub fn new() -> Self {
        Self {
            kalman: Kalman::<3>::new(),
            state: FlightState::Idle,
            last_update: Instant::now(),
            imu_orientation: UnitQuaternion::new_normalize(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            last_state_transition: Instant::now()
        }
    }

    pub fn update(&mut self, altitude: f32, acceleration: Vector3<f32>) {
        let now = Instant::now();
        let dt: f32 = now.duration_since(self.last_update).as_millis() as f32 / 1000.0;

        // accel stuff
        let accel_norm = acceleration.norm();
        if accel_norm < Self::GRAVITY_UPPER_THRESHOLD && accel_norm > Self::GRAVITY_LOWER_THRESHOLD {
            // TODO: low pass filter this
            self.imu_orientation = Self::calc_imu_orientation_correction(acceleration);
        }
        let rotated_acceleration: Vector3<f32> = self.imu_orientation * acceleration;
        let vertical_acceleration: f32 = rotated_acceleration.z - Self::GRAVITY;

        // prepare kalman process
        let a = kalman::AMatrix::<3>::new(
            1.0, dt, dt * dt / 2.0,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0
        );

        let q;
        if self.state == FlightState::Descent {
            q = Self::calc_q_matrix(dt, Self::SIGMA_PROCESS_ACCEL_DESCENT);
        } else {
            q = Self::calc_q_matrix(dt, Self::SIGMA_PROCESS_ACCEL_ASCENT);
        }

        self.kalman.set_process(&q, &a);

        // prepare kalman measurement
        let r = kalman::RMatrix::<3>::new(
            Self::SIGMA_MEASUREMENT_ALTITUDE, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, Self::SIGMA_MEASUREMENT_ACCEL
        );

        let z = kalman::ZMatrix::<3>::new(
            altitude,
            0.0,
            vertical_acceleration
        );

        // during the descent phase the acceleration is pointless so disregard it
        // literally no point in using a kalman filter then but yolo
        let h;
        if self.state == FlightState::Descent {
            h = kalman::HMatrix::<3>::new(
                1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            );
        } else {
            h = kalman::HMatrix::<3>::new(
                1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 1.0
            );
        }
        self.kalman.insert_measurement(&z, &r, &h);


        // evaluate state
        let old_state = self.state.clone();
        match self.state {
            FlightState::Idle => {
                if self.get_vertical_acceleration() > Self::IDLE_ACCELERATION_THRESHOLD {
                    self.state = FlightState::MaybeLaunched;
                }
            }

            FlightState::MaybeLaunched => {
                let in_state_for: f32 = Instant::now().duration_since(self.last_state_transition).as_millis() as f32 / 1000.0;
                if self.get_altitude() > Self::MAYBE_LAUNCHED_ALTITUDE_THRESHOLD {
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
                }
            }

            FlightState::Descent => {
                let in_state_for: f32 = Instant::now().duration_since(self.last_state_transition).as_millis() as f32 / 1000.0;
                if in_state_for > Self::DESCENT_HOLDOFF && self.get_vertical_velocity() < 0.0 {
                    self.state = FlightState::Idle;
                }
            }
        }

        if old_state != self.state {
            self.last_state_transition = Instant::now();
        }
    }

    pub fn get_state(&self) -> &FlightState {
        &self.state
    }

    pub fn get_altitude(&self) -> f32{
        self.kalman.get_state()[0]
    }

    pub fn get_vertical_velocity(&self) -> f32{
        self.kalman.get_state()[1]
    }

    pub fn get_vertical_acceleration(&self) -> f32 {
        self.kalman.get_state()[2]
    }


    // noise from acceleration is the dominating factor. We can extrapolate the rest from that
    fn calc_q_matrix(dt: f32, sigma_accel: f32) -> kalman::QMatrix<3> {
        let q = kalman::QMatrix::<3>::new(
            dt.powi(4) / 4.0,   dt.powi(3) / 2.0,   dt.powi(2) / 2.0,
            dt.powi(3) / 2.0,   dt.powi(2),         dt,
            dt.powi(2) / 2.0,   dt,                 1.0
        );
        q * sigma_accel
    }

    fn calc_imu_orientation_correction(measured: Vector3<f32>) -> UnitQuaternion<f32> {
        // Normalize the measured vector
        let mut measured_vector = measured.normalize();
        // Calculate rotation around x-axis
        let x_rot_angle = measured_vector.y.atan2(measured_vector.z) / 2.0;
        let x_rot_quat = UnitQuaternion::from_quaternion(Quaternion::new(x_rot_angle.sin(), 0.0, 0.0, x_rot_angle.cos()));

        // Apply x-axis rotation to the measured vector and normalize
        measured_vector = x_rot_quat * measured_vector;
        measured_vector = measured_vector.normalize();

        // Calculate rotation around y-axis
        let y_rot_angle = -measured_vector.x.atan2(measured_vector.z) / 2.0;
        let y_rot_quat = UnitQuaternion::from_quaternion(Quaternion::new(y_rot_angle.cos(), 0.0, y_rot_angle.sin(), 0.0));

        // Combine the rotations to get the final orientation
        let result = y_rot_quat * x_rot_quat;
        result
    }
}