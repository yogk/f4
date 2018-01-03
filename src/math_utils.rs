//! Some math utilities

use core::mem;
use m::Float as _0;

/// Convert radians to degrees by multiplying with this number
pub const RAD_TO_DEG: f32 = 57.29578;
/// Convert degrees to radians by multiplying with this number
pub const DEG_TO_RAD: f32 = 0.0174533;

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

    /// Calculate the euler angles from the quaternion.
    pub fn to_euler_angles(&self) -> Vector3<f32> {
        let q0 = self.x;
        let q1 = self.y;
        let q2 = self.z;
        let q3 = self.w;

        let yaw: f32 = (q0 * q1 + q2 * q3).atan2(0.5 - q1 * q1 - q2 * q2);
        let roll: f32 = (q1 * q2 + q0 * q3).atan2(0.5 - q2 * q2 - q3 * q3);
        let y = -2.0 * (q1 * q3 - q0 * q2);
        let x = 1.0 + (1.0 - y * y).sqrt();
        let pitch = 2.0 * y.atan2(x);

        Vector3 {
            x: roll * RAD_TO_DEG,
            y: pitch * RAD_TO_DEG,
            z: yaw * RAD_TO_DEG,
        }
    }

    /// The squared length
    pub fn len2(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    /// Normalize
    pub fn nor(&self) -> Quaternion<f32> {
        let recip_norm = fast_inv_sqrt(self.len2());
        Quaternion {
            x: self.x * recip_norm,
            y: self.y * recip_norm,
            z: self.z * recip_norm,
            w: self.w * recip_norm,
        }
    }

    /// Add another quaternion to this one
    pub fn add(&self, other: Quaternion<f32>) -> Quaternion<f32> {
        Quaternion {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            w: self.w + other.w,
        }
    }

    /// Subtract another quaternion to this one
    pub fn sub(&self, other: Quaternion<f32>) -> Quaternion<f32> {
        Quaternion {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            w: self.w - other.w,
        }
    }

    /// Scale
    pub fn scl(&self, s: f32) -> Quaternion<f32> {
        Quaternion {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
            w: self.w * s,
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
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// The squared length
    pub fn len2(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Normalize
    pub fn nor(&self) -> Vector3<f32> {
        let recip_norm = fast_inv_sqrt(self.len2());
        self.scl(recip_norm)
    }

    /// Scale
    pub fn scl(&self, s: f32) -> Vector3<f32> {
        Vector3 {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }

    /// Add another vector to this one
    pub fn add(&self, other: Vector3<f32>) -> Vector3<f32> {
        Vector3 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    /// Subtract another vector to this one
    pub fn sub(&self, other: Vector3<f32>) -> Vector3<f32> {
        Vector3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    /// Multiply by a block diagonal matrix in which the diagonals are equal to the other vector
    pub fn mul(&self, other: Vector3<f32>) -> Vector3<f32> {
        Vector3 {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
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
