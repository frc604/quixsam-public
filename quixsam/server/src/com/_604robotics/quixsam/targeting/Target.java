package com._604robotics.quixsam.targeting;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Target {
    private final Translation2d centerPosition;
    private final double centerHeight;
    private final Landmark[] landmarks;

    public Target(Translation2d centerPosition, double centerHeight, Landmark... landmarks) {
        this.centerPosition = centerPosition;
        this.centerHeight = centerHeight;
        this.landmarks = landmarks;
    }

    public Translation2d getCenterPosition() {
        return centerPosition;
    }

    public double getCenterHeight() {
        return centerHeight;
    }

    public Landmark[] getLandmarks() {
        return landmarks;
    }
}