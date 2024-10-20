// SHUT UP SHUT UP SHUT UP SHUT UP SHUT UP SHUT UP SHUT UP SHUT UP SHUT UP
#![allow(dead_code)]

use core::convert::identity;

use nalgebra as na;




// this didn't work lol
pub type AMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type HMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type RMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type QMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type ZMatrix<const DIM: usize> = na::SMatrix<f32, DIM, 1>;
pub type XMatrix<const DIM: usize> = na::SMatrix<f32, DIM, 1>;
pub type KMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type PMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;


/// x: The current state
/// p: Current errors
/// k: current gains
pub struct Kalman<const DIM: usize> {
    x: XMatrix<DIM>,
    p: PMatrix<DIM>,
    k: KMatrix<DIM>,

    q: QMatrix<DIM>,
    a: AMatrix<DIM>,

    prev_t: f32,
}

impl<const DIM: usize> Kalman<DIM> {
    pub fn new() -> Self {
        // initial states, shouldn't matter unless you want it to converge really fast
        Self {
            p: PMatrix::<DIM>::identity() * 100_f32,
            x: XMatrix::<DIM>::zeros(),
            k: KMatrix::<DIM>::zeros(),

            q: QMatrix::<DIM>::identity(),
            a: AMatrix::<DIM>::identity(),

            prev_t: 0_f32,
        }
    }

    pub fn update_process(&mut self,
        q: &QMatrix<DIM>, // process covariance
        a: &AMatrix<DIM> // how the state transitions (i.e. vel += acc)
    ) {
        self.q = *q;
        self.a = *a;
    }

    pub fn get_kalman_gain(&self) -> KMatrix<DIM> {
        self.k
    }

    pub fn get_process_covariance(&self) -> PMatrix<DIM> {
        self.p
    }

    pub fn get_state(&self) -> XMatrix<DIM> {
        self.x
    }

    pub fn insert_measurement(&mut self,
        t: &f32, // curent time
        z: &ZMatrix<DIM>, // measurement vector
        r: &RMatrix<DIM>, // Noise in the measurement (typically diagonal matrix but only on indexes with measurements)
        h: &HMatrix<DIM>, // Maps the measurement onto the state (typically identity matrix)
    ) {
        let dt: f32;
        if self.prev_t == 0.0 {
            dt = 0.0;
        } else {
            dt = t - self.prev_t;
        }
        self.prev_t = *t;


        // prediction step
        self.x = self.a * self.x * dt;
        self.p = self.a * self.p * self.a.transpose() + self.q;


        // measurement step
        let s: na::SMatrix<f32, DIM, DIM> = h * self.p * h.transpose() + r;
        let s_inv: na::SMatrix<f32, DIM, DIM> = s.try_inverse().expect("Couldn't invert s");

        // kalman gain
        self.k = self.p * h.transpose() * s_inv; // Kalman gain

        // update state with measurement
        let y = z - (h * self.x);
        self.x = self.x + self.k * y;

        // update error
        self.p = (RMatrix::<DIM>::identity() - self.k * h) * self.p;
    }
}