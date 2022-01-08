#pragma once

#include <vector>
#include <Eigen/Core>

using namespace Eigen;

namespace quixpf {

class ParticleFilter {
    public:
        ParticleFilter(int numParticles, const Matrix<double, 3, 1>& prior, const Matrix<double, 3, 1>& priorSigma);

        void Initialize(int numParticles, const Matrix<double, 3, 1>& prior, const Matrix<double, 3, 1>& priorSigma);

        void Predict(double dt, const Matrix<double, 2, 1>& u, const Matrix<double, 2, 1>& uSigma);

        void Update(const Matrix<double, 2, 1>& measurement, const Matrix<double, 3, 1>& landmark, const Matrix<double, 2, 1>& measurementSigma);

        void Resample();

        MatrixXd getParticles() {
            return particles;
        };

        static Matrix<double, 2, 1> Cart2BESph(const Matrix<double, 3, 1>& point) {
            Matrix<double, 2, 1> result(std::atan2(point(1), point(0)), std::atan2(point(2), std::sqrt(std::pow(point(0), 2) + std::pow(point(1), 2))));
            return result;
        }

        static Matrix<double, 3, 1> PointRelTo2DPose(const Matrix<double, 3, 1>& point, const Matrix<double, 3, 1>& pose) {
            Matrix<double, 4, 1> poseAsPoint;
            poseAsPoint << pose(0), pose(1), 0.0, 0.0;

            Matrix<double, 4, 1> extendedPoint;
            extendedPoint << point(0), point(1), point(2), 1.0;
            
            Matrix<double, 4, 4> rotationMatrix;
            rotationMatrix << std::cos(pose(2)), -std::sin(pose(2)), 0.0, 0.0,
                              std::sin(pose(2)), std::cos(pose(2)), 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0;

            return (rotationMatrix * (extendedPoint - poseAsPoint)).head(3);
        }

    private:
        int numParticles;

        MatrixXd particles;

        Matrix<double, 3, 1> Dynamics(const Matrix<double, 3, 1>& x, const Matrix<double, 2, 1>& u) {
            return Matrix<double, 3, 1>(
                u(0) * std::cos(x(2)),
                u(0) * std::sin(x(2)),
                u(1)
            );
        };

};
}