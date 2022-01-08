package com._604robotics.robotnik.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Swervetest {

    public static void main(String[] args) {
        for (int i = 0; i < 360; i++) {
            var desiredAngle = ((Math.random() * 2) - 1) * 1000;
            var currentAngle = ((Math.random() * 2) - 1) * 1000;
    
            var desiredSpeed = ((Math.random() * 2) - 1) * 100;
    
            var desiredState = new QuixSwerveModuleState(desiredSpeed, Rotation2d.fromDegrees(desiredAngle));
            var desiredState2 = new QuixSwerveModuleState2(desiredSpeed, Rotation2d.fromDegrees(desiredAngle));
            System.out.println("------------------------------------------------");
            System.out.println(currentAngle);
            System.out.println(desiredState);
            System.out.println(QuixSwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(currentAngle)));
            System.out.println(QuixSwerveModuleState2.optimize(desiredState2, Rotation2d.fromDegrees(currentAngle)));
            System.out.println("------------------------------------------------");
        }
    }
    
}
