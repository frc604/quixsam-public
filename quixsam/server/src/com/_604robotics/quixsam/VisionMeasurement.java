package com._604robotics.quixsam;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class VisionMeasurement {
  private Rotation2d bearing;
  private Rotation2d elevation;

  public VisionMeasurement(Rotation2d bearing, Rotation2d elevation) {
    this.bearing = bearing;
    this.elevation = elevation;
  }

  public Rotation2d getElevation() {
    return elevation;
  }

  public Rotation2d getBearing() {
    return bearing;
  }

  public void setBearing(Rotation2d bearing) {
    this.bearing = bearing;
  }

  public void setElevation(Rotation2d elevation) {
    this.elevation = elevation;
  }
}
