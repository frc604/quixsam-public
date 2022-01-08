package com._604robotics.quixsam.mathematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

public class InterpolatablePose2d extends Interpolatable<Pose2d> {
    public InterpolatablePose2d(Pose2d value) {
        super(value);
    }

    @Override
    public Pose2d interpolate(Pose2d endValue, double t) {
      if (t < 0) {
        return this.get();
      } else if (t >= 1) {
        return endValue;
      } else {
        var twist = super.value.log(endValue);
        return new Pose2d().exp(new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t));
      }
    }
}
