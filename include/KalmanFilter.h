#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
    public:
        // model state
        float h;    // altitude above ground (m)
        float v;    // vertical velocity (m/s, + is up)
        float b_a;  // accelerometer bias (m/s/s, IMU systematic error)

    private:
        float P[3][3];    // covarience matrix (uncertainty of state)

        const float R;    // baro measurement noise varience (m^2)

        const float stddev_a;
        const float stddev_b;

    public:
        KalmanFilter(
            const float x_init[3],      // [altitude, vertical velocity, accelerometer bias]
            const float P_init[3][3],
            float R_,
            float stddev_a_,
            float stddev_b_
        );

        void Predict(float dt, float a_imu);
        void Update(float z_baro);
};

#endif
