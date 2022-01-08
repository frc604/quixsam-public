#include <random>
#include<iostream>
#include <cmath>
#include <Eigen/LU>
#include <algorithm>
#include <omp.h>

#include "particle_filter.h"


using namespace quixpf;
using namespace std;

ParticleFilter::ParticleFilter(int numParticles, const Matrix<double, 3, 1>& prior, const Matrix<double, 3, 1>& priorSigma) {
    Initialize(numParticles, prior, priorSigma);
}

void ParticleFilter::Initialize(int numParticles, const Matrix<double, 3, 1>& prior, const Matrix<double, 3, 1>& priorSigma) {
    this->numParticles = numParticles;
    particles.resize(numParticles, 4);

    std::default_random_engine generator;

    std::normal_distribution<double> xDistribution(prior(0), priorSigma(0));
    std::normal_distribution<double> yDistribution(prior(1), priorSigma(1));
    std::normal_distribution<double> thetaDistribution(prior(2), priorSigma(2));

    #pragma omp parallel for
    for (int i = 0; i < numParticles; i++) {
        particles(i, 0) = xDistribution(generator);
        particles(i, 1) = yDistribution(generator);
        particles(i, 2) = thetaDistribution(generator);
    }

    particles.col(3).fill(1.0);
}

void ParticleFilter::Predict(double dt, const Matrix<double, 2, 1>& u, const Matrix<double, 2, 1>& uSigma) {
    std::default_random_engine generator;

    std::normal_distribution<double> vDistribution(u(0), uSigma(0));
    std::normal_distribution<double> wDistribution(u(1), uSigma(1));

    #pragma omp parallel for
    for (int i = 0; i < numParticles; i++) {
        Matrix<double, 2, 1> noisyU;
        noisyU << vDistribution(generator), wDistribution(generator);
        particles.row(i).head(3) += Dynamics(particles.row(i).head(3), noisyU) * dt;
    }
}

void ParticleFilter::Update(const Matrix<double, 2, 1>& measurement, const Matrix<double, 3, 1>& landmark, const Matrix<double, 2, 1>& measurementSigma) {
    VectorXd likelihoods(numParticles);

    #pragma omp parallel for
    for (int i = 0; i < numParticles; i++) {
        Matrix<double, 3, 1> rotatedPoint = PointRelTo2DPose(landmark, particles.row(i).head(3));

        Matrix<double, 2, 1> estimatedMeasurement = Cart2BESph(rotatedPoint);

        Matrix<double, 2, 2> covarianceMatrix = measurementSigma.asDiagonal();
        Matrix<double, 2, 1> deltaMeasurement = estimatedMeasurement - measurement;

        double scalarTerm = ( 1.0 / std::sqrt(std::pow(2.0 * M_PI, 2) * covarianceMatrix.determinant()));
        double mainTerm = -0.5 * (deltaMeasurement.transpose() * covarianceMatrix.inverse()) * deltaMeasurement;

        likelihoods(i) = scalarTerm * std::exp(mainTerm);
    }
    particles.col(3) = likelihoods.normalized();
}

void ParticleFilter::Resample() {
    std::default_random_engine generator;
    
    MatrixXd tempParticles = particles;

    std::vector<double> weightVector(&particles.col(3)[0], particles.col(3).data() + particles.col(3).cols() * particles.col(3).rows());
    std::discrete_distribution<> weightDistribution(weightVector.begin(), weightVector.end());

    #pragma omp parallel for
    for (int i = 0; i < numParticles; i++) {
        particles.row(i) = tempParticles.row(weightDistribution(generator));
    }
}