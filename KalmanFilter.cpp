/* This module is responsible for implementing a KALMAN FILTER, allowing for altitude 
estimations at higher frequencies than the barometer can measure using accurate estimation
via IMU data & previous baro reading.

INPUTS
    Precomputed
        - unmodelled acceleration   (stddev_a)
        - bias drift rate           (stddev_b) 
        - barometer noise varience  (R)
        - initial covarience matrix (P_init)
        - initial state             (x_init)
    Live
        - IMU vertical accel        (a_imu)
        - barometer altitude        (z_baro)

OUTPUTS
    - estimated altitude            (x[0], h)
    - estimated vertical vel.       (x[1], v)
*/

class KalmanFilter {
    public:
        // model state
        float h;    // altitude above ground (m)
        float v;    // vertical velocity (m/s, + is up)
        float b_a;  // accelerometer bias (m/s/s, IMU systematic error)

    private:
        float P[3][3];    // covarience matrix (uncertainty of state)
        float q0, q1, q2; // diagonal of process noise matrix

        const float R;    // baro measurement noise varience (m²)

        const float stddev_a;
        const float stddev_b;

    public:
        KalmanFilter(
            const float x_init[3],      // [altitude, vertical velocity, accelerometer bias]
            const float P_init[3][3],
            float R_,
            float stddev_a_,
            float stddev_b_
        ) : R(R_), stddev_a(stddev_a_), stddev_b(stddev_b_) {
        
            h   = x_init[0];
            v   = x_init[1];
            b_a = x_init[2];
            
            // initialise P, Q matricies
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    P[i][j] = P_init[i][j];
        }

        void Predict(float dt, float a_imu) {
            float dt2 = dt*dt;
            float tmp[3][3];

            // recompute Q with dt
            float q0 = stddev_a * stddev_a * (dt2*dt2) / 4.0f;
            float q1 = stddev_a * stddev_a * dt2;
            float q2 = stddev_b * stddev_b * dt;

            // update state
            float u = a_imu - b_a;      // drift corrected IMU reading

            h += v * dt + 0.5f * u * dt * dt;   // update altitude
            v += u * dt;                        // update velocity
            // bias does not change

            // update state certainty (F x P x Fᵀ + Q)

            // F = [ 1, dt, -dt*dt/2 ]
            //     [ 0,  1,    -dt   ]
            //     [ 0,  0,      1   ]

            // tmp = F x P
            tmp[0][0] = P[0][0] + dt*P[1][0] - (dt2/2)*P[2][0];
            tmp[0][1] = P[0][1] + dt*P[1][1] - (dt2/2)*P[2][1];
            tmp[0][2] = P[0][2] + dt*P[1][2] - (dt2/2)*P[2][2];

            tmp[1][0] = P[1][0] - dt*P[2][0];
            tmp[1][1] = P[1][1] - dt*P[2][1];
            tmp[1][2] = P[1][2] - dt*P[2][2];

            tmp[2][0] = P[2][0];
            tmp[2][1] = P[2][1];
            tmp[2][2] = P[2][2];

            // P = tmp x Fᵀ + Q
            P[0][0] = tmp[0][0] + dt*tmp[0][1] - (dt2/2)*tmp[0][2] + q0;
            P[1][0] = tmp[1][0] + dt*tmp[1][1] - (dt2/2)*tmp[1][2];
            P[2][0] = tmp[2][0] + dt*tmp[2][1] - (dt2/2)*tmp[2][2];

            P[0][1] = tmp[0][1] - dt*tmp[0][2];
            P[1][1] = tmp[1][1] - dt*tmp[1][2]  + q1;
            P[2][1] = tmp[2][1] - dt*tmp[2][2];

            P[0][2] = tmp[0][2];
            P[1][2] = tmp[1][2];
            P[2][2] = tmp[2][2]  + q2;

            // enforce symmetry
            P[1][0] = P[0][1];
            P[2][0] = P[0][2];
            P[2][1] = P[1][2];
        }

        void Update(float z_baro) {
            float y = z_baro - h; // error from prediction in m
            float S = P[0][0] + R;

            // calculate reliance on barometer / IMU readings in future
            // 1 = high barometer weight
            // 0 = high IMU weight

            float K_h = P[0][0] / S;
            float K_v = P[1][0] / S;
            float K_ba = P[2][0] / S;
            
            // correct state
            h += K_h * y;
            v += K_v * y;
            b_a += K_ba * y;

            // update P matrix
           float P0[3] = { P[0][0], P[0][1], P[0][2] }; // snapshot previous values

            P[0][0] -= K_h  * P0[0];
            P[0][1] -= K_h  * P0[1];
            P[0][2] -= K_h  * P0[2];

            P[1][0] -= K_v  * P0[0];
            P[1][1] -= K_v  * P0[1];
            P[1][2] -= K_v  * P0[2];

            P[2][0] -= K_ba * P0[0];
            P[2][1] -= K_ba * P0[1];
            P[2][2] -= K_ba * P0[2];

            // enforce symmetry
            P[1][0] = P[0][1];
            P[2][0] = P[0][2];
            P[2][1] = P[1][2];
        }
};