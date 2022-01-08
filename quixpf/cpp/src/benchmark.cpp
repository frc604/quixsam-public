#include <benchmark/benchmark.h>
#include<iostream>
#include <Eigen/Core>

#include "particle_filter.h"

using namespace std;
using namespace quixpf;

static void BM_FULLSTACKPF(benchmark::State& state) {
  Matrix<double, 3, 1> priori;
  priori << 0.0, 0.0, 0.0;

  Matrix<double, 3, 1> prioriSigma;
  prioriSigma << 0.1, 0.1, 0.001;

  Matrix<double, 3, 1> landmark;
  landmark << 3, 0, 0;

  Matrix<double, 2, 1> measurementSigma;
  measurementSigma << 0.01, 0.01;

  ParticleFilter filter(100000, priori, prioriSigma);

  double x = 0.0;
  for (auto _ : state)
   for (int i = 1; i < 4; i++) {
      Vector2d u;
      u << i, 0;

      Vector2d uSigma;
      uSigma << 0.005, 0.005;

      filter.Predict(1.0, u, uSigma);

      x += i;

      Vector3d pose;
      pose << x, 0, 0;

      filter.Update(ParticleFilter::Cart2BESph(ParticleFilter::PointRelTo2DPose(landmark, pose)), landmark, measurementSigma);
      filter.Resample();
   }
}
// Register the function as a benchmark
BENCHMARK(BM_FULLSTACKPF);

BENCHMARK_MAIN();