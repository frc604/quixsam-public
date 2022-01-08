package com._604robotics.quixsam.odometry;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class VisionMeasurement {
    private Rotation2d elevation;
    private Rotation2d bearing;

    private Rotation2d sigmaBearing;
    private Rotation2d sigmaElevation;

    public VisionMeasurement(Rotation2d elevation, Rotation2d bearing, Rotation2d sigmaBearing, Rotation2d sigmaElevation) {
        this.bearing = bearing;
        this.elevation = elevation;
        this.sigmaBearing = sigmaBearing;
        this.sigmaElevation = sigmaElevation;
    }
    
    public Rotation2d getBearing() {
        return bearing;
    }

    public Rotation2d getElevation() {
        return elevation;
    }

    public Rotation2d getSigmaBearing() {
        return sigmaBearing;
    }

    public Rotation2d getSigmaElevation() {
        return sigmaElevation;
    }
}
