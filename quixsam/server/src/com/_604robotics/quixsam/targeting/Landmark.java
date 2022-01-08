package com._604robotics.quixsam.targeting;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Landmark {
    private final Translation2d position;
    private final double height;

    public Landmark(Translation2d position, double height) {
        this.position = position;
        this.height = height;
    }

    public Translation2d getPosition() {
        return position;
    }

    public double getHeight() {
        return height;
    }
}
