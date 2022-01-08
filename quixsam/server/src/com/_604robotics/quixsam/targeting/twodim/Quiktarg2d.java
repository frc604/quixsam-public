package com._604robotics.quixsam.targeting.twodim;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Quiktarg2d {
    private Transform2d robotToCamera;
    private Rotation2d horizontialFOV;
    private Rotation2d verticalFOV;
    private double rangeEstimate;

    public Quiktarg2d(Rotation2d horizontialFOV, Rotation2d verticalFOV, double rangeEstimate, Transform2d robotToCamera) {
        this.robotToCamera = robotToCamera;
        this.horizontialFOV = horizontialFOV;
        this.verticalFOV = verticalFOV;
        this.rangeEstimate = rangeEstimate;
    }

    public Pose2d update(Pose2d robotPose) {
        return robotPose.transformBy(robotToCamera);
    }

    public Rotation2d getAngle(Pose2d robotPose, Translation2d targetTranslation) {
        Pose2d cameraPose = robotPose.transformBy(robotToCamera);
        return new Rotation2d(Math.atan2(targetTranslation.getY() - cameraPose.getY(), targetTranslation.getX() - cameraPose.getX())).minus(cameraPose.getRotation());
    }

}
