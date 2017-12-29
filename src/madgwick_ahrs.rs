//! Rust implementation of Madgwick's IMU and AHRS algorithms.
//! See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

use math_utils;
use math_utils::Quaternion;
use m::Float as _0;

/// Madgwick's IMU and AHRS
pub struct MadgwickAhrs {
    /// Quaternion of sensor frame relative to auxiliary frame
    q: Quaternion<f32>,
    beta: f32, // 2 * proportional gain (Kp)
    sample_freq: f32,
}

///
impl MadgwickAhrs {
    /// Begin filtering with specified sample frequency in Hz
    pub const fn begin(sample_freq: f32) -> Self {
        MadgwickAhrs {
            q: Quaternion::new(),
            beta: 0.1, // 2 * proportional gain
            sample_freq: sample_freq,
        }
    }

    /// The filter should be updated at the frequency specified in begin()
    pub fn madgwick_ahrs_update_imu(
        &mut self,
        gx: f32,
        gy: f32,
        gz: f32,
        ax: f32,
        ay: f32,
        az: f32,
    ) -> Quaternion<f32> {
        let mut recip_norm: f32;

        let mut ax = ax;
        let mut ay = ay;
        let mut az = az;

        let mut gx = gx;
        let mut gy = gy;
        let mut gz = gz;

        // Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533;
        gy *= 0.0174533;
        gz *= 0.0174533;

        let q0 = self.q.x;
        let q1 = self.q.y;
        let q2 = self.q.z;
        let q3 = self.q.w;

        // Rate of change of quaternion from gyroscope
        let mut q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let mut q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let mut q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let mut q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
            // Normalise accelerometer measurement
            recip_norm = math_utils::fast_inv_sqrt(ax * ax + ay * ay + az * az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            // Auxiliary variables to avoid repeated arithmetic
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _4q0 = 4.0 * q0;
            let _4q1 = 4.0 * q1;
            let _4q2 = 4.0 * q2;
            let _8q1 = 8.0 * q1;
            let _8q2 = 8.0 * q2;
            let q0q0 = q0 * q0;
            let q1q1 = q1 * q1;
            let q2q2 = q2 * q2;
            let q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            let mut s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let mut s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1
                + _8q1 * q2q2 + _4q1 * az;
            let mut s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1
                + _8q2 * q2q2 + _4q2 * az;
            let mut s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
            recip_norm = math_utils::fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recip_norm;
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;

            // Apply feedback step
            q_dot1 -= self.beta * s0;
            q_dot2 -= self.beta * s1;
            q_dot3 -= self.beta * s2;
            q_dot4 -= self.beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        self.q.x += q_dot1 * (1.0 / self.sample_freq);
        self.q.y += q_dot2 * (1.0 / self.sample_freq);
        self.q.z += q_dot3 * (1.0 / self.sample_freq);
        self.q.w += q_dot4 * (1.0 / self.sample_freq);

        // Normalise quaternion
        recip_norm = math_utils::fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        self.q.x *= recip_norm;
        self.q.y *= recip_norm;
        self.q.z *= recip_norm;
        self.q.w *= recip_norm;

        Quaternion {
            x: self.q.x,
            y: self.q.y,
            z: self.q.z,
            w: self.q.w,
        }
    }

    /// The filter should be updated at the frequency specified in begin()
    pub fn madgwick_ahrs_update(
        &mut self,
        gx: f32,
        gy: f32,
        gz: f32,
        ax: f32,
        ay: f32,
        az: f32,
        mx: f32,
        my: f32,
        mz: f32,
    ) -> Quaternion<f32> {
        let mut recip_norm: f32;
        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if (mx == 0.0) && (my == 0.0) && (mz == 0.0) {
            return self.madgwick_ahrs_update_imu(gx, gy, gz, ax, ay, az);
        }

        let mut ax = ax;
        let mut ay = ay;
        let mut az = az;

        let mut gx = gx;
        let mut gy = gy;
        let mut gz = gz;

        let mut mx = mx;
        let mut my = my;
        let mut mz = mz;

        // Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533;
        gy *= 0.0174533;
        gz *= 0.0174533;

        let q0 = self.q.x;
        let q1 = self.q.y;
        let q2 = self.q.z;
        let q3 = self.q.w;

        // Rate of change of quaternion from gyroscope
        let mut q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
        let mut q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
        let mut q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
        let mut q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
            // Normalise accelerometer measurement
            recip_norm = math_utils::fast_inv_sqrt(ax * ax + ay * ay + az * az);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            // Normalise magnetometer measurement
            recip_norm = math_utils::fast_inv_sqrt(mx * mx + my * my + mz * mz);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;

            // Auxiliary variables to avoid repeated arithmetic
            let _2q0mx = 2.0 * q0 * mx;
            let _2q0my = 2.0 * q0 * my;
            let _2q0mz = 2.0 * q0 * mz;
            let _2q1mx = 2.0 * q1 * mx;
            let _2q0 = 2.0 * q0;
            let _2q1 = 2.0 * q1;
            let _2q2 = 2.0 * q2;
            let _2q3 = 2.0 * q3;
            let _2q0q2 = 2.0 * q0 * q2;
            let _2q2q3 = 2.0 * q2 * q3;
            let q0q0 = q0 * q0;
            let q0q1 = q0 * q1;
            let q0q2 = q0 * q2;
            let q0q3 = q0 * q3;
            let q1q1 = q1 * q1;
            let q1q2 = q1 * q2;
            let q1q3 = q1 * q3;
            let q2q2 = q2 * q2;
            let q2q3 = q2 * q3;
            let q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            let hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2
                + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            let hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2
                + _2q2 * mz * q3 - my * q3q3;
            let _2bx = (hx * hx + hy * hy).sqrt();
            let _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1
                + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            let _4bx = 2.0 * _2bx;
            let _4bz = 2.0 * _2bz;

            // Gradient decent algorithm corrective step
            let mut s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
                - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
            let mut s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
                - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
                + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + (_2bx * q3 - _4bz * q1)
                    * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
            let mut s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
                - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
                + (-_4bx * q2 - _2bz * q0)
                    * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + (_2bx * q0 - _4bz * q2)
                    * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
            let mut s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
                + (-_4bx * q3 + _2bz * q1)
                    * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);

            recip_norm = math_utils::fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude

            s0 *= recip_norm;
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;

            // Apply feedback step
            q_dot1 -= self.beta * s0;
            q_dot2 -= self.beta * s1;
            q_dot3 -= self.beta * s2;
            q_dot4 -= self.beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        self.q.x += q_dot1 * (1.0 / self.sample_freq);
        self.q.y += q_dot2 * (1.0 / self.sample_freq);
        self.q.z += q_dot3 * (1.0 / self.sample_freq);
        self.q.w += q_dot4 * (1.0 / self.sample_freq);

        // Normalise quaternion
        recip_norm = math_utils::fast_inv_sqrt(
            self.q.x * self.q.x + self.q.y * self.q.y + self.q.y * self.q.y + self.q.w * self.q.w,
        );
        self.q.x *= recip_norm;
        self.q.y *= recip_norm;
        self.q.z *= recip_norm;
        self.q.w *= recip_norm;

        Quaternion {
            x: self.q.x,
            y: self.q.y,
            z: self.q.z,
            w: self.q.w,
        }
    }
}
