use nalgebra as na;

#[allow(unused)]
use defmt::{info};
// this didn't work as I would've hoped (the type system doesn't differentiate)
// but it is still a moderately useful
pub type AMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type QMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;
pub type XMatrix<const DIM: usize> = na::SMatrix<f32, DIM, 1>;
pub type PMatrix<const DIM: usize> = na::SMatrix<f32, DIM, DIM>;

pub type KMatrix<const DIM: usize, const MEAS_DIM: usize> = na::SMatrix<f32, DIM, MEAS_DIM>;
pub type ZMatrix<const DIM: usize, const MEAS_DIM: usize> = na::SMatrix<f32, MEAS_DIM, 1>;
pub type RMatrix<const DIM: usize, const MEAS_DIM: usize> = na::SMatrix<f32, MEAS_DIM, MEAS_DIM>;
pub type HMatrix<const DIM: usize, const MEAS_DIM: usize> = na::SMatrix<f32, MEAS_DIM, DIM>;

/// x: The current state
/// p: Current errors
/// k: current gains
pub struct Kalman<const DIM: usize> {
    x: XMatrix<DIM>,
    p: PMatrix<DIM>,

    q: QMatrix<DIM>,
    a: AMatrix<DIM>,
}

impl<const DIM: usize> Kalman<DIM> {
    pub fn new() -> Self {
        // initial states, shouldn't matter unless you want it to converge really fast
        Self {
            p: PMatrix::<DIM>::identity() * 100_f32,
            x: XMatrix::<DIM>::zeros(),

            q: QMatrix::<DIM>::identity(),
            a: AMatrix::<DIM>::identity(),
        }
    }

    pub fn set_process(&mut self,
        q: &QMatrix<DIM>, // process covariance
        a: &AMatrix<DIM> // how the state transitions (i.e. vel += acc)
    ) {
        self.q = *q;
        self.a = *a;
    }

    #[allow(unused)]
    pub fn get_process_covariance(&self) -> PMatrix<DIM> {
        self.p
    }

    pub fn get_state(&self) -> XMatrix<DIM> {
        self.x
    }

    pub fn insert_measurement<const MEAS_DIM: usize>(&mut self,
        z: &ZMatrix<DIM, MEAS_DIM>, // measurement vector
        r: &RMatrix<DIM, MEAS_DIM>, // Noise in the measurement (typically diagonal matrix but only on indexes with measurements)
        h: &HMatrix<DIM, MEAS_DIM>, // Maps the measurement onto the state (typically identity matrix)
    ) {
        // predict
        let x_p: XMatrix<DIM> = self.a * self.x;
        let p_p: PMatrix<DIM> = self.a * self.p * self.a.transpose() + self.q;

        // update
        let y = z - (h * x_p);
        let s: na::SMatrix<f32, MEAS_DIM, MEAS_DIM> = h * p_p * h.transpose() + r;
        let s_inv: na::SMatrix<f32, MEAS_DIM, MEAS_DIM> = s.try_inverse().expect("couldn't invert s");
        let k: KMatrix<DIM, MEAS_DIM> = p_p * h.transpose() * s_inv; // Kalman gain

        // update state with measurement
        self.x = x_p + k * y;
        self.p = (PMatrix::<DIM>::identity() - k * h) * p_p;
    }
}