//! Some math utilities

use core::mem;
use m::Float as _0;

/// Convert radians to degrees by multiplying with this number
pub const RAD_TO_DEG: f32 = 57.29578;

/// Generic 4D vector
#[derive(Clone, Copy, Debug)]
pub struct Quaternion<T> {
    /// X axis value
    pub x: T,
    /// Y axis value
    pub y: T,
    /// Z axis value
    pub z: T,
    /// W axis value
    pub w: T,
}

impl<T> Quaternion<T> {
    /// Set this quaternion from another
    pub fn set(&mut self, other: Quaternion<T>) {
        self.x = other.x;
        self.y = other.y;
        self.z = other.z;
        self.w = other.w;
    }
}

impl Quaternion<f32> {
    /// Creates a new normalized quaternion
    pub const fn new() -> Self {
        Quaternion {
            x: 1.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        }
    }

    /// Calculate the euler angles from the quaternion. TODO
    pub fn to_euler_angles(&self) -> Vector3<f32> {
        let q0 = self.x;
        let q1 = self.y;
        let q2 = self.z;
        let q3 = self.w;

        let x = 2.0 * (q1 * q3 - q0 * q2);
        let y = 2.0 * (q0 * q1 + q2 * q3);
        let z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        let pitch = x.atan2((y * y + z * z).sqrt());
        let yaw = y.atan2((x * x + z * z).sqrt());
        let roll = (2.0 * q1 * q2 - 2.0 * q0 * q3).atan2(2.0 * q0 * q0 + 2.0 * q1 * q1 - 1.0);

        // let roll: f32 = (q0 * q1 + q2 * q3).atan2(0.5 - q1 * q1 - q2 * q2);
        // // let pitch = asinf(-2.0 * (q1 * q3 - q0 * q2));
        // let y = -2.0 * (q1 * q3 - q0 * q2);
        // let x = 1.0 + (1.0 - y * y).sqrt();
        // let pitch = 2.0 * y.atan2(x);
        // let yaw: f32 = (q1 * q2 + q0 * q3).atan2(0.5 - q2 * q2 - q3 * q3);

        Vector3 {
            x: roll * RAD_TO_DEG,
            y: pitch * RAD_TO_DEG,
            z: yaw * RAD_TO_DEG,
        }
    }
}

/// Generic 3D vector
#[derive(Clone, Copy, Debug)]
pub struct Vector3<T> {
    /// X axis value
    pub x: T,
    /// Y axis value
    pub y: T,
    /// Z axis value
    pub z: T,
}

impl<T> Vector3<T> {
    /// Set this vector from another
    pub fn set(&mut self, other: Vector3<T>) {
        self.x = other.x;
        self.y = other.y;
        self.z = other.z;
    }
}

impl Vector3<f32> {
    /// Creates a new normalized vector
    pub const fn new() -> Self {
        Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }
    }
}
impl Vector3<i16> {
    /// Creates a new normalized vector
    pub const fn new() -> Self {
        Vector3 { x: 1, y: 0, z: 0 }
    }
}
/// Fast inverse square-root
/// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
pub fn fast_inv_sqrt(x: f32) -> f32 {
    let i: u32 = unsafe { mem::transmute(x) };
    let j = 0x5f3759df - (i >> 1);
    let y: f32 = unsafe { mem::transmute(j) };
    y * (1.5 - 0.5 * x * y * y)
}
