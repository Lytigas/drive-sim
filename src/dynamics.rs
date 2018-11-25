//! https://www.omicsonline.org/open-access/dynamic-modelling-of-differentialdrive-mobile-robots-using-lagrange-and-newtoneuler-methodologies-a-unified-framework-2168-9695.1000107.pdf

use dimensioned::si::*;
use dimensioned::tarr;
use dimensioned::typenum::{N1, N2, P1, P2, Z0};
use std::fmt::Debug;
use std::ops::{Add, Div, Mul, Sub};

#[derive(Debug, Clone)]
pub struct Integrator<U>
where
    U: Add<tarr![Z0, Z0, P1, Z0, Z0, Z0, Z0]>,
    <SI<f64, U> as Mul<Second<f64>>>::Output: Debug + Copy + Clone,
{
    dt: Second<f64>,
    acc: <SI<f64, U> as Mul<Second<f64>>>::Output,
}

impl<U> Integrator<U>
where
    U: Add<tarr![Z0, Z0, P1, Z0, Z0, Z0, Z0]>,
    <SI<f64, U> as Mul<Second<f64>>>::Output: Debug + Copy + Clone,
{
    pub fn new(dt: Second<f64>, initial: <SI<f64, U> as Mul<Second<f64>>>::Output) -> Self {
        Self { dt, acc: initial }
    }

    pub fn get(&self) -> <SI<f64, U> as Mul<Second<f64>>>::Output {
        self.acc
    }

    pub fn add(&mut self, val: SI<f64, U>) -> <SI<f64, U> as Mul<Second<f64>>>::Output {
        self.acc += val.mul(self.dt);
        self.get()
    }
}

#[test]
fn integrator() {
    let mut i = Integrator::new(0.005 * S, 0. * M);
    i.add(1.0 * MPS);
    i.add(0.05 * MPS);
    assert_eq!(i.get(), 0.005 * (1.0 + 0.05) * M);
}

#[derive(Debug, Clone)]
pub struct Differentiator<U>
where
    U: Sub<tarr![Z0, Z0, P1, Z0, Z0, Z0, Z0]>, // seconds
    SI<f64, U>: Copy + Clone + Debug,
{
    dt: Second<f64>,
    last: SI<f64, U>,
    two: SI<f64, U>,
}

impl<U> Differentiator<U>
where
    U: Sub<tarr![Z0, Z0, P1, Z0, Z0, Z0, Z0]>, // seconds
    SI<f64, U>: Copy + Clone + Debug,
{
    pub fn new(dt: Second<f64>, initial: SI<f64, U>) -> Self {
        Self {
            dt,
            last: initial,
            two: initial,
        }
    }

    pub fn get(&self) -> <SI<f64, U> as Div<Second<f64>>>::Output {
        (self.last - self.two) / self.dt
    }

    pub fn add(&mut self, val: SI<f64, U>) -> <SI<f64, U> as Div<Second<f64>>>::Output {
        self.two = self.last;
        self.last = val;
        self.get()
    }
}

#[test]
fn differentiator() {
    use dimensioned::traits::Abs;
    let mut d = Differentiator::new(0.005 * S, 0. * M);
    assert_eq!(d.get(), 0.0 * MPS);
    d.add(1.0 * M);
    assert!((d.add(1.2 * M) - 40. * MPS).abs() < 0.0001 * MPS);
}

/// Moment of Inertia
pub type KilogramMeter2<V> = SI<V, tarr![P2, P1, Z0, Z0, Z0, Z0, Z0]>;
/// Torque constant
pub type NewtonMeterPerAmpere<V> = SI<V, tarr![P2, P1, N2, N1, Z0, Z0, Z0]>;
/// Back-emf constant
pub type VoltSecond<V> = SI<V, tarr![P2, P1, N2, N1, Z0, Z0, Z0]>;
/// Torque
pub type NewtonMeter<V> = SI<V, tarr![P2, P1, N2, Z0, Z0, Z0, Z0]>;

#[allow(non_snake_case)]
pub struct DDMRParams {
    /// R = wheel radius
    pub R: Meter<f64>,
    /// M = total mass of the robot including wheels and actuators
    pub m: Kilogram<f64>,
    /// M_c = mass of the robot without wheels and actuators
    pub mc: Kilogram<f64>,
    /// d = the distance behind the center of mass of the wheels
    pub d: Meter<f64>,
    /// L = half the wheel base
    pub L: Meter<f64>,
    /// I = the moment of inertia of the entire robot about its center of rotation when driving (see the paper for calculation details)
    pub I: KilogramMeter2<f64>,
    /// Iw = the moment of inertia of each wheel about the wheel axis
    pub Iw: KilogramMeter2<f64>,
}

#[allow(non_snake_case)]
pub struct DCMotorParams {
    /// Ra = armature resistance
    pub Ra: Ohm<f64>,
    /// La = armature inductance
    pub La: Henry<f64>,
    /// N = gear ratio such that `rotor_ang_vel = N * wheel_ang_vel`
    pub N: f64,
    /// Kb = Back-EMF constant, such that `e_a = K_b * w_m`
    pub Kb: VoltSecond<f64>,
    /// Kt = Torque constance, such that `tau_m = K_t * i_a`
    pub Kt: NewtonMeterPerAmpere<f64>,
}

#[derive(Debug, Copy, Clone)]
pub struct Vels {
    pub lin: MeterPerSecond<f64>,
    /// Only in hertz because angular velocity is measured in `rad / s = 1 / s`
    pub ang: Hertz<f64>,
}

impl Default for Vels {
    fn default() -> Self {
        Self {
            lin: 0. * MPS,
            ang: 0. / S,
        }
    }
}

#[derive(Debug, Copy, Clone, Default)]
pub struct LR<T> {
    pub l: T,
    pub r: T,
}

type Acceleration = tarr![P1, Z0, N2, Z0, Z0, Z0, Z0];
type AngularAcceleration = tarr![Z0, Z0, N2, Z0, Z0, Z0, Z0];
type Current = tarr![Z0, Z0, Z0, P1, Z0, Z0, Z0];

pub struct DDMRModel {
    p: DDMRParams,
    linv: Integrator<Acceleration>,
    angv: Integrator<AngularAcceleration>,
}

impl DDMRModel {
    pub fn new(dt: Second<f64>, param: DDMRParams) -> Self {
        Self {
            p: param,
            linv: Integrator::new(dt, 0. * MPS),
            angv: Integrator::new(dt, 0. * HZ),
        }
    }

    pub fn vel(&self) -> Vels {
        Vels {
            lin: self.linv.get(),
            ang: self.angv.get(),
        }
    }

    // equation 47
    pub fn observe(&mut self, tau: LR<NewtonMeter<f64>>) -> Vels {
        let p = &self.p;
        let vdot: MeterPerSecond2<f64> = ((tau.r + tau.l) / p.R
            + p.mc * p.d * self.angv.get() * self.angv.get())
            / (p.m + 2. * p.Iw / p.R / p.R);
        let wdot: SI<f64, AngularAcceleration> = ((tau.r - tau.l) * p.L / p.R
            - p.mc * p.d * self.angv.get() * self.linv.get())
            / (p.I + 2. * p.L * p.L * p.Iw / p.R / p.R);

        Vels {
            lin: self.linv.add(vdot),
            ang: self.angv.add(wdot),
        }
    }

    pub fn vels_to_wheel(&self, v: Vels) -> LR<Hertz<f64>> {
        LR {
            l: (v.lin - self.p.L * v.ang) / self.p.R,
            r: (v.lin + self.p.L * v.ang) / self.p.R,
        }
    }

    pub fn wheels(&self) -> LR<Hertz<f64>> {
        self.vels_to_wheel(self.vel())
    }
}

pub struct ActuatedDDMRModel {
    ddmr: DDMRModel,
    p: DCMotorParams,
    di: LR<Differentiator<Current>>,
}

impl ActuatedDDMRModel {
    pub fn new(dt: Second<f64>, ddmr_par: DDMRParams, params: DCMotorParams) -> Self {
        Self {
            ddmr: DDMRModel::new(dt, ddmr_par),
            p: params,
            di: LR {
                l: Differentiator::new(dt, 0. * A),
                r: Differentiator::new(dt, 0. * A),
            },
        }
    }

    pub fn observe(&mut self, v: LR<Volt<f64>>) -> Vels {
        let p = &self.p;
        let phidot = self.ddmr.wheels();
        let ial: Ampere<f64> = (v.l - p.Kb * p.N * phidot.l - p.La * self.di.l.get()) / p.Ra;
        let iar: Ampere<f64> = (v.r - p.Kb * p.N * phidot.r - p.La * self.di.r.get()) / p.Ra;
        self.di.l.add(ial);
        self.di.r.add(iar);
        self.ddmr.observe(LR {
            l: ial * p.Kt,
            r: iar * p.Kt,
        })
    }
}
