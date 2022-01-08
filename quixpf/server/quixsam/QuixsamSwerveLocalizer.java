package com._604robotics.quixsam;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.NavigableSet;
import java.util.Set;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;

import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.quixsam.odometry.QuixSwerveDriveOdometry;
import com._604robotics.quixsam.odometry.SendableOdometryMeasurment;
import com._604robotics.quixsam.vision.SendableVisionMeasurment;
import com._604robotics.quixsam.odometry.SwerveDriveOdometryMeasurement;
import com._604robotics.robotnik.prefabs.vision.VisionCamera;
import com._604robotics.robotnik.prefabs.vision.VisionCamera.Target;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpiutil.math.Pair;

public class QuixsamSwerveLocalizer {
    private int currentID = 0;
    private HashMap<Integer, Double> idMap = new HashMap<>();

    private TreeMap<Double, SwerveDriveOdometryMeasurement> odometryMap = new TreeMap<>();
    private DoubleInterpolatableTreeMap<Pose2d> poseMap = new DoubleInterpolatableTreeMap<>();

    private TreeMap<Double, Pair<SendableOdometryMeasurment, SendableVisionMeasurment>> buffer = new TreeMap<>();

    private QuixSwerveDriveOdometry rawOdometry;
    private QuixSwerveDriveOdometry playbackOdometry;

    private QuixsamNetworkTable networkTable;

    private double timeTreshold = 1; // seconds

    public QuixsamSwerveLocalizer(String name, SwerveDriveKinematics kinematics, Pose2d priori, Pose2d prioriSigma, Rotation2d initialGyroAngle) {
        rawOdometry = new QuixSwerveDriveOdometry(kinematics, initialGyroAngle, priori);
        playbackOdometry = new QuixSwerveDriveOdometry(kinematics, initialGyroAngle, priori);

        networkTable = new QuixsamNetworkTable(name, priori, prioriSigma, this::computeEstimate);
    }

    public void update(SwerveDriveOdometryMeasurement odometry, VisionCamera.PipelineVisionPacket vision) {
        currentID += 1;
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        playbackOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        odometryMap.put(currentTime, odometry);

        poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

        double visionTime = currentTime - (vision.getLatency() / 1000);
        Pose2d interpolatedPose = poseMap.get(visionTime);

        SendableOdometryMeasurment sendableOdometryMeasurment;
        SendableVisionMeasurment sendableVisionMeasurment = null;

        Target bestTarget = vision.getBestTarget();
        if (bestTarget.getCorners().size() == 4) {
            sendableVisionMeasurment = new SendableVisionMeasurment(currentID);

            for (Pair<Double, Double> corner : bestTarget.getCorners()) {
                sendableVisionMeasurment.addMeasurment(new Pair<>(corner.getFirst(), corner.getSecond()), new Pair<>(0.1, 0.1));
            }

            sendableOdometryMeasurment = new SendableOdometryMeasurment(currentID, interpolatedPose, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
        } else {
            sendableOdometryMeasurment = new SendableOdometryMeasurment(currentID, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)));
        }


        idMap.put(currentID, visionTime);
        buffer.put(currentTime, new Pair<>(sendableOdometryMeasurment, sendableVisionMeasurment));
    }

    public void update(SwerveDriveOdometryMeasurement odometry) {
        currentID += 1;
        double currentTime = Timer.getFPGATimestamp();

        rawOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        playbackOdometry.updateWithTime(currentTime, odometry.getGyroAngle(), odometry.getModuleStates());
        odometryMap.put(currentTime, odometry);

        poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

        idMap.put(currentID, currentTime);
        buffer.put(currentTime, new Pair<>(new SendableOdometryMeasurment(currentID, rawOdometry.getPoseMeters(), new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1))), null));
    }

    public void computeEstimate(QuixsamEsimate estimate) {
        int estimateID = estimate.getID();
        double estimateTime = idMap.get(estimateID);

        playbackOdometry.resetPosition(estimate.getPose(), odometryMap.get(estimateTime).getGyroAngle());

        Double lastKey = odometryMap.ceilingKey(estimateTime); //least key >= time

        double prevTime = odometryMap.lowerKey(lastKey);
        while (lastKey != null) {
            SwerveDriveOdometryMeasurement lastMeasurment = odometryMap.get(lastKey);
            playbackOdometry.updateWithTime(prevTime, lastKey, lastMeasurment.getGyroAngle(), lastMeasurment.getModuleStates());
            prevTime = lastKey;
            odometryMap.remove(lastKey);
            lastKey = odometryMap.ceilingKey(lastKey);
        }
    }

    public Pose2d getPose() {
        return playbackOdometry.getPoseMeters();
    }

    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();

        ArrayList<Double> keys = new ArrayList<>(buffer.keySet());
        for (double key : keys) {
            if (currentTime - key > timeTreshold) {
                networkTable.publishOdometry(buffer.get(key).getFirst());
                System.out.println(buffer.get(key).getFirst().getPose());
                if (buffer.get(key).getSecond() != null) {
                    networkTable.publishVision(buffer.get(key).getSecond());
                }
                buffer.remove(key);
            }
        }
    }
}
