//! Rust implementation of Madgwick's IMU and AHRS algorithms.
//! 
//! See: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

use math_utils::{Quaternion, Vector3};
use m::Float as _0;

/// Madgwick's IMU and AHRS
/// 
/// See: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
pub struct MadgwickAhrs {
    /// Quaternion of sensor frame relative to auxiliary frame
    q: Quaternion<f32>,
    beta: f32, // 2 * proportional gain (Kp)
    inv_sample_freq: f32,
}

///
impl MadgwickAhrs {
    /// Begin filtering with specified sample frequency in Hz
    pub const fn begin(sample_freq: f32) -> Self {
        MadgwickAhrs {
            q: Quaternion::new(),
            beta: 0.1, // 2 * proportional gain
            inv_sample_freq: 1.0 / sample_freq,
        }
    }

    /// The filter should be updated at the frequency specified in begin()
    fn update_imu(&mut self, a: Vector3<f32>, g: Vector3<f32>) -> Quaternion<f32> {
        let q = self.q;

        // Rate of change of quaternion from gyroscope
        let mut q_dot: Quaternion<f32> = Quaternion {
            x: 0.5 * (-q.y * g.x - q.z * g.y - q.w * g.z),
            y: 0.5 * (q.x * g.x + q.z * g.z - q.w * g.y),
            z: 0.5 * (q.x * g.y - q.y * g.z + q.w * g.x),
            w: 0.5 * (q.x * g.z + q.y * g.y - q.z * g.x),
        };

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if !((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0)) {
            // Normalise accelerometer measurement
            let a = a.nor();

            // Auxiliary variables to avoid repeated arithmetic
            let _2q0 = 2.0 * q.x;
            let _2q1 = 2.0 * q.y;
            let _2q2 = 2.0 * q.z;
            let _2q3 = 2.0 * q.w;
            let _4q0 = 4.0 * q.x;
            let _4q1 = 4.0 * q.y;
            let _4q2 = 4.0 * q.z;
            let _8q1 = 8.0 * q.y;
            let _8q2 = 8.0 * q.z;
            let q0q0 = q.x * q.x;
            let q1q1 = q.y * q.y;
            let q2q2 = q.z * q.z;
            let q3q3 = q.w * q.w;

            // Gradient decent algorithm corrective step
            let s: Quaternion<f32> = Quaternion {
                x: _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y,
                y: _4q1 * q3q3 - _2q3 * a.x + 4.0 * q0q0 * q.y - _2q0 * a.y - _4q1 + _8q1 * q1q1
                    + _8q1 * q2q2 + _4q1 * a.z,
                z: 4.0 * q0q0 * q.z + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1
                    + _8q2 * q2q2 + _4q2 * a.z,
                w: 4.0 * q1q1 * q.w - _2q1 * a.x + 4.0 * q2q2 * q.w - _2q2 * a.y,
            };
            // Apply feedback step
            q_dot = q_dot.sub(s.nor().scl(self.beta));
        }

        // Integrate rate of change of quaternion to yield quaternion
        self.q.set(q.add(q_dot.scl(self.inv_sample_freq)).nor());
        self.q.clone()
    }

    /// The filter should be updated at the frequency specified in begin()
    pub fn update(
        &mut self,
        accelerometer: Vector3<f32>,
        gyroscope: Vector3<f32>,
        magnetometer: Option<Vector3<f32>>,
    ) -> Quaternion<f32> {
        let a = accelerometer;
        let g = gyroscope;
        let m = magnetometer;

        if let Some(m) = m {
            let q = self.q;

            // Rate of change of quaternion from gyroscope
            let mut q_dot: Quaternion<f32> = Quaternion {
                x: 0.5 * (-q.y * g.x - q.z * g.y - q.w * g.z),
                y: 0.5 * (q.x * g.x + q.z * g.z - q.w * g.y),
                z: 0.5 * (q.x * g.y - q.y * g.z + q.w * g.x),
                w: 0.5 * (q.x * g.z + q.y * g.y - q.z * g.x),
            };
            // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
            if !((a.x == 0.0) && (a.y == 0.0) && (a.z == 0.0)) {
                // Normalise accelerometer and magnetometer measurement
                let a = a.nor();
                let m = m.nor();

                // Auxiliary variables to avoid repeated arithmetic
                let _2qxmx = 2.0 * q.x * m.x;
                let _2qxmy = 2.0 * q.x * m.y;
                let _2qxmz = 2.0 * q.x * m.z;
                let _2qymx = 2.0 * q.y * m.x;
                let _2qx = 2.0 * q.x;
                let _2qy = 2.0 * q.y;
                let _2qz = 2.0 * q.z;
                let _2qw = 2.0 * q.w;
                let _2qxqz = 2.0 * q.x * q.z;
                let _2qxqw = 2.0 * q.z * q.w;
                let qxqx = q.x * q.x;
                let qxqy = q.x * q.y;
                let qxqz = q.x * q.z;
                let qxqw = q.x * q.w;
                let qyqy = q.y * q.y;
                let qyqz = q.y * q.z;
                let qyqw = q.y * q.w;
                let qzqz = q.z * q.z;
                let qzqw = q.z * q.w;
                let qwqw = q.w * q.w;

                // Reference direction of Earth's magnetic field
                let hx = m.x * qxqx - _2qxmy * q.w + _2qxmz * q.z + m.x * qyqy + _2qy * m.y * q.z
                    + _2qy * m.z * q.w - m.x * qzqz - m.x * qwqw;
                let hy = _2qxmx * q.w + m.y * qxqx - _2qxmz * q.y + _2qymx * q.z - m.y * qyqy
                    + m.y * qzqz + _2qz * m.z * q.w - m.y * qwqw;
                let _2bx = (hx * hx + hy * hy).sqrt();
                let _2bz = -_2qxmx * q.z + _2qxmy * q.y + m.z * qxqx + _2qymx * q.w - m.z * qyqy
                    + _2qz * m.y * q.w - m.z * qzqz + m.z * qwqw;
                let _4bx = 2.0 * _2bx;
                let _4bz = 2.0 * _2bz;

                // Gradient decent algorithm corrective step
                let s: Quaternion<f32> = Quaternion {
                    x: -_2qz * (2.0 * qyqw - _2qxqz - a.x) + _2qy * (2.0 * qxqy + _2qxqw - a.y)
                        - _2bz * q.z * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
                        + (-_2bx * q.w + _2bz * q.y)
                            * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
                        + _2bx * q.z * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
                    y: _2qw * (2.0 * qyqw - _2qxqz - a.x) + _2qx * (2.0 * qxqy + _2qxqw - a.y)
                        - 4.0 * q.y * (1.0 - 2.0 * qyqy - 2.0 * qzqz - a.z)
                        + _2bz * q.w * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
                        + (_2bx * q.z + _2bz * q.x)
                            * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
                        + (_2bx * q.w - _4bz * q.y)
                            * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
                    z: -_2qx * (2.0 * qyqw - _2qxqz - a.x) + _2qw * (2.0 * qxqy + _2qxqw - a.y)
                        - 4.0 * q.z * (1.0 - 2.0 * qyqy - 2.0 * qzqz - a.z)
                        + (-_4bx * q.z - _2bz * q.x)
                            * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
                        + (_2bx * q.y + _2bz * q.w)
                            * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
                        + (_2bx * q.x - _4bz * q.z)
                            * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
                    w: _2qy * (2.0 * qyqw - _2qxqz - a.x) + _2qz * (2.0 * qxqy + _2qxqw - a.y)
                        + (-_4bx * q.w + _2bz * q.y)
                            * (_2bx * (0.5 - qzqz - qwqw) + _2bz * (qyqw - qxqz) - m.x)
                        + (-_2bx * q.x + _2bz * q.z)
                            * (_2bx * (qyqz - qxqw) + _2bz * (qxqy + qzqw) - m.y)
                        + _2bx * q.y * (_2bx * (qxqz + qyqw) + _2bz * (0.5 - qyqy - qzqz) - m.z),
                };
                // Apply feedback step
                q_dot = q_dot.sub(s.nor().scl(self.beta));
            }

            // Integrate rate of change of quaternion to yield quaternion
            self.q.set(q.add(q_dot.scl(self.inv_sample_freq)).nor());
            self.q.clone()
        } else {
            // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
            return self.update_imu(a, g);
        }
    }
}
