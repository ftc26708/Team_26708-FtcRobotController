package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Locale;
import java.util.Optional;

public class Localization {
    private final Limelight3A limelight;
    private final Follower follower;
    private static final double METERS_TO_INCHES = 39.3701;
    private static final double MOUNT_HEIGHT = 0.26490; // Meters
    private static final double TARGET_HEIGHT = 0.74930; // Meters
    private static final double MOUNT_ANGLE = 13.42; // Degrees
    private static final double LIMELIGHT_X_OFFSET = 0.154 * METERS_TO_INCHES; // forward, to be measured and changed
    private static final double LIMELIGHT_Y_OFFSET = 0.004 * METERS_TO_INCHES;  // left
    public final Pose BLUE_GOAL_POSE_AIM = new Pose(6, 138, 0);
    public final Pose RED_GOAL_POSE_AIM = new Pose(138, 138, 0);
    public final Pose BLUE_GOAL_POSE_RELOC = new Pose(72 - 55.6425, 72 + 58.3727, 0);
    public final Pose RED_GOAL_POSE_RELOC = new Pose(72 + 55.6425, 72 + 58.3727, 0);
    private int pipeline = -1;

    public Localization(Follower follower, Limelight3A limelight) {
        this.follower = follower;
        this.limelight = limelight;
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        this.pipeline = pipeline;
    }
    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }
    public void setOdometryPose(Pose pose) {
        follower.setPose(pose);
    }
    public Pose getOdometryPose() {
        return follower.getPose();
    }
    // In Localization.java
    public Optional<Pose> getCameraPose(Pose targetAprilTag) {
        LLResult result = limelight.getLatestResult();

        // Check if result is valid before doing math
        if (result == null || !result.isValid()) {
            return Optional.empty();
        }

        double ty = result.getTy();
        double tx = result.getTx();

        // Math remains the same, just using the targetAprilTag passed in
        double totalAngleRadians = Math.toRadians(MOUNT_ANGLE + ty);
        double visionDistanceInches = ((TARGET_HEIGHT - MOUNT_HEIGHT) / Math.tan(totalAngleRadians)) * METERS_TO_INCHES;

        Pose currentPose = follower.getPose(); // Use the follower reference you already have
        double angleToGoalField = currentPose.getHeading() - Math.toRadians(tx);

        double camX = targetAprilTag.getX() - (Math.cos(angleToGoalField) * visionDistanceInches);
        double camY = targetAprilTag.getY() - (Math.sin(angleToGoalField) * visionDistanceInches);

        // Robot Center Offset
        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());
        double newX = camX - (LIMELIGHT_X_OFFSET * cosH - LIMELIGHT_Y_OFFSET * sinH);
        double newY = camY - (LIMELIGHT_X_OFFSET * sinH + LIMELIGHT_Y_OFFSET * cosH);

        return Optional.of(new Pose(newX, newY, currentPose.getHeading()));
    }
    public double getDistanceToPose(Pose targetPose) {
        Pose currentPose = follower.getPose();

        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        return Math.hypot(dx, dy);
    }
    public double getAngleToPose(Pose targetPose) {
        Pose currentPose = follower.getPose();

        // 1. Calculate the field-centric angle to the target
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double angleToTarget = Math.atan2(dy, dx); // Result is in Radians

        // 2. Calculate the difference relative to robot heading
        double relativeAngle = angleToTarget - currentPose.getHeading();

        // 3. Normalize the angle to be between -pi and pi
        // This prevents the robot from turning 270 degrees instead of -90
        while (relativeAngle > Math.PI)  relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        // 4. Convert to Degrees for PID logic
        return Math.toDegrees(relativeAngle);
    }

    public String getTelemetry() {
        Pose p = follower.getPose();
        Vector v = follower.getVelocity();
        boolean locked = limelight.getLatestResult().isValid();
        return String.format(Locale.US,
                "LOC: (%.1f, %.1f, %.1f°)s" +
                        "\nVEL: %.1f in/s, %1f degrees" +
                        "\nCAM: %s | Pipe: %d",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()),
                v.getXComponent(), v.getYComponent(),
                locked ? "LOCKED" : "SEARCHING", pipeline
        );
    }
}
