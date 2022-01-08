#include<iostream>
#include <Eigen/Core>

#include "matplotlibcpp.h"
#include "particle_filter.h"

using namespace std;
using namespace quixpf;
namespace plt = matplotlibcpp;
 
int main(int argc, char *argv[]){
   Matrix<double, 3, 1> priori;
   priori << 0.0, 0.0, 0.0;

   Matrix<double, 3, 1> prioriSigma;
   prioriSigma << 0.1, 0.1, 0.001;

   Matrix<double, 3, 1> landmark;
   landmark << 3, 0, 0;

   Matrix<double, 2, 1> measurementSigma;
   measurementSigma << 0.01, 0.01;

   ParticleFilter filter(10000, priori, prioriSigma);

   VectorXd xs = filter.getParticles().col(0);
   VectorXd ys = filter.getParticles().col(1);

   std::vector<double> xVector(&xs[0], xs.data() + xs.cols() * xs.rows());
   std::vector<double> yVector(&ys[0], ys.data() + ys.cols() * ys.rows());

   plt::axis("equal");   
   plt::scatter(xVector, yVector);

   double x = 0.0;

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

      xs = filter.getParticles().col(0);
      ys = filter.getParticles().col(1);

      std::vector<double> xVector(&xs[0], xs.data() + xs.cols() * xs.rows());
      std::vector<double> yVector(&ys[0], ys.data() + ys.cols() * ys.rows());

      plt::scatter(xVector, yVector);

      xVector.clear();
      yVector.clear();
   }

   plt::show();
   
}