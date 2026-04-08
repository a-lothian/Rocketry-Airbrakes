#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "KalmanFilter.h"

namespace py = pybind11;

PYBIND11_MODULE(kalman_filter, m) {
    m.doc() = "pybind11 wrapper for KalmanFilter";

    py::class_<KalmanFilter>(m, "KalmanFilter")
        .def(py::init([](py::list x_init, py::list P_init,
                         float R, float stddev_a, float stddev_b) {
            float x[3];
            for (int i = 0; i < 3; i++)
                x[i] = x_init[i].cast<float>();

            float P[3][3];
            for (int i = 0; i < 3; i++) {
                py::list row = P_init[i];
                for (int j = 0; j < 3; j++)
                    P[i][j] = row[j].cast<float>();
            }

            return new KalmanFilter(x, P, R, stddev_a, stddev_b);
        }), py::arg("x_init"), py::arg("P_init"),
            py::arg("R"), py::arg("stddev_a"), py::arg("stddev_b"))

        .def("predict", &KalmanFilter::Predict,
             py::arg("dt"), py::arg("a_imu"))
        .def("update", &KalmanFilter::Update,
             py::arg("z_baro"))

        .def_readonly("h",   &KalmanFilter::h)
        .def_readonly("v",   &KalmanFilter::v)
        .def_readonly("b_a", &KalmanFilter::b_a);
}
