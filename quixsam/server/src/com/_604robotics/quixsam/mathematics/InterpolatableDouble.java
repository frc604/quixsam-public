package com._604robotics.quixsam.mathematics;

public class InterpolatableDouble extends Interpolatable<Double> {
    public InterpolatableDouble(double value) {
        super(value);
    }

    @Override
    public Double interpolate(Double endValue, double t) {
      return (1 - t) * super.value + t * endValue; // Lerp
    }
}
