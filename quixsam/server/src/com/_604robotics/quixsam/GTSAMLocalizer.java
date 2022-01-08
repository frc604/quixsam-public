package com._604robotics.quixsam;

import java.util.HashMap;

import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.quixsam.mathematics.InterpolatableTreeMap;
import com._604robotics.quixsam.odometry.DiffDriveOdometryMeasurement;
import com._604robotics.quixsam.odometry.VisionMeasurement;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpiutil.math.Pair;

public class GTSAMLocalizer {
    private int currentID = 0;
    private HashMap<Integer, Double> idMap;

    private HashMap<Double, DiffDriveOdometryMeasurement> odometryMap;
    private DoubleInterpolatableTreeMap<Pose2d> poseMap = new DoubleInterpolatableTreeMap<>();

    private DifferentialDriveOdometry rawOdometry;
    private DifferentialDriveOdometry playbackOdometry;

    public GTSAMLocalizer(Pose2d poseMeters, Rotation2d gyroAngle) {
        rawOdometry = new DifferentialDriveOdometry(gyroAngle, poseMeters);
        playbackOdometry = new DifferentialDriveOdometry(gyroAngle, poseMeters);
    }

    public void update(DiffDriveOdometryMeasurement odometry, VisionMeasurement vision) {
        currentID +=1
        double currentTime = Timer.getFPGATimestamp();
        odometry.addTo(rawOdometry);
        odometry.addTo(playbackOdometry);

        odometryMap.put(currentTime, odometry);

        poseMap.set(currentTime, rawOdometry.getPoseMeters());

        visionTime = vision.getTime()
        interpolatedPose = Interpolate odometryMap[visionTime]
        
        idMap.add(currentID, visionTime)
        Send [currentID, interpolatedPose + bearingElevationVision] to GTSAM
        currentID++
        idMap.add(currentID, currentTime)
        Send [currentID, rawOdometry.pose()] to GTSAM
    }

    public void update(DiffDriveOdometryMeasurement odometry) {
        currentID++
        currentTime = getTime()
        Add odometry to rawOdometry

        Add odometry to playBackOdometry
        Add [currentTime, odometry] to odometryMap

        Add [currrentTime, rawOdometry.pose()] to poseMap

        idMap.add(currentID, currentTime)
        Send [currentID, rawOdometry.pose()] to GTSAM
    }

    public void computeEstimate(newGTSAMEstimate) {
        estimateID = newGTSAMEstimate.getID()
        estimateTime = idMap.getValue(estimateID)

        playbackOdometry.reset(newGTSAMEstimate.pose)

        lastKey = odometryMap.ceilingKey(estimateTime) //least key >= time

        while lastkey != null:
            playbackOdometry.update(odometryMap.get(lastKey))
            lastKey = odometryMap.ceilingKey(lastKey)
    }

    public Pose2d getPose() {
        return playbackOdometry.getPoseMeters();
    }
}
