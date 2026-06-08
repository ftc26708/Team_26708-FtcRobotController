package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Optional;

public class Robot {
    private final Follower follower;
    private final Drivetrain drivetrain;
    private final IntakeTransfer intakeTransfer;
    private final Shooter shooter;
    private final Localization localization;
    public static class DataPasser {
        public enum Alliance {
            RED,
            BLUE,
            UNKNOWN
        }
        public static Alliance currentAlliance = Alliance.UNKNOWN;
        public static Pose endAutoPose = new Pose(72, 72, Math.toRadians(90));
        private static long actionStartTime = 0;
        public static void resetTimer() {
            actionStartTime = System.nanoTime();
        }
        public static double getElapsed() {
            return (System.nanoTime() - actionStartTime) / 1e9;
        }
        public static boolean hasElapsed(double seconds) {
            return getElapsed() >= seconds;
        }
    }
    private final double AIM_KP = 0.030;
    private final double AIM_KF = 0.015;
    private final double MAX_TURN = 0.6;
    private Pose goalPose;
    private Pose relocalizationPose;
    List<LynxModule> allHubs;
    private long lastLoopTime = 0;
    private double currentLoopTimeMs = 0;
    public Robot(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(follower);
        intakeTransfer = new IntakeTransfer(
                hardwareMap.get(DcMotorEx.class, "IN"),
                hardwareMap.get(DcMotorEx.class, "TR")
        );
        shooter = new Shooter(
                hardwareMap.get(DcMotorEx.class, "LS"),
                hardwareMap.get(DcMotorEx.class, "RS")
        );
        localization = new Localization(
                follower,
                hardwareMap.get(Limelight3A.class, "LM")
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clearCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    // LOCALIZATION
    public void setAlliance(DataPasser.Alliance alliance) {
        DataPasser.currentAlliance = alliance;
        if (alliance == DataPasser.Alliance.RED) {
            localization.setPipeline(1);
            goalPose = localization.RED_GOAL_POSE_AIM;
            relocalizationPose = localization.RED_GOAL_POSE_RELOC;
        } else {
            localization.setPipeline(0);
            goalPose = localization.BLUE_GOAL_POSE_AIM;
            relocalizationPose = localization.BLUE_GOAL_POSE_RELOC;
        }
    }
    public boolean relocalizeOdoWithCamera() {
        Optional<Pose> cameraPose = localization.getCameraPose(relocalizationPose);
        if (cameraPose.isPresent()) {
            setPose(cameraPose.get());
            return true;
        }
        return false;
    }
    public void setStartingPose(Pose startingPose) {
        localization.setStartingPose(startingPose);
    }
    public void startTeleopDrive() {
        drivetrain.startTeleopDrive();
    }
    public void setPose(Pose pose) {
        localization.setOdometryPose(pose);
    }
    public Pose getPose() {
        return localization.getOdometryPose();
    }

    // DRIVETRAIN
    public void updateDrivetrain() {
        drivetrain.update();
    }
    public void manualDrive(double x, double y, double turn) {
        drivetrain.drive(x, y, turn);
    }
    public void autoAimDrive(double x, double y) {
        drivetrain.drive(
                x, y,
                Math.max(-MAX_TURN, Math.min(MAX_TURN,
                        (localization.getAngleToPose(goalPose) * AIM_KP) +
                                (Math.signum(localization.getAngleToPose(goalPose)) * AIM_KF)
                        )
                )
        );
    }
    public PathBuilder pathBuilder() {
        return drivetrain.pathBuilder();
    }
    public void followPath(Path path) {
        drivetrain.followPath(path);
    }
    public void followPath(PathChain pathChain) {
        drivetrain.followPath(pathChain);
    }
    public void followPath(Path path, boolean holdEnd) {
        drivetrain.followPath(path, holdEnd);
    }
    public void followPath(PathChain pathChain, boolean holdEnd) {
        drivetrain.followPath(pathChain, holdEnd);
    }
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        drivetrain.followPath(pathChain, maxPower, holdEnd);
    }
    public boolean isNotPathFollowing() {
        return !drivetrain.isBusy();
    }
    public void breakPathFollowing() {
        drivetrain.breakFollowing();
    }
    public void setFieldCentric(boolean fieldCentric) {
        drivetrain.setFieldCentric(fieldCentric);
    }
    public boolean getFieldCentric() {
        return drivetrain.getFieldCentric();
    }

    // ARTIFACT MOVEMENT
    public void intake(double speed) {
        intakeTransfer.moveIntake(speed);
        intakeTransfer.moveTransfer(speed);
        shooter.setTargetVelocity(-600);
    }
    public void prepareSpinUp() {
        intakeTransfer.positionIntake(-0.125);
        intakeTransfer.positionTransfer(-0.125);
        shooter.setTargetVelocity(-600);
    }
    public void spinUp() {
        intakeTransfer.moveIntake(-0.125);
        intakeTransfer.moveTransfer(-0.125);
        shooter.setTargetVelocity(shooter.getNeededVelocity(localization.getDistanceToPose(goalPose)));
    }
    public void shoot() {
        intakeTransfer.moveIntake(1);
        intakeTransfer.moveTransfer(1);
        shooter.setTargetVelocity(shooter.getNeededVelocity(localization.getDistanceToPose(goalPose)));
    }
    public void stop() {
        intakeTransfer.moveIntake(0);
        intakeTransfer.moveTransfer(0);
        shooter.setTargetVelocity(0);
    }
    public void customMechSpeeds(double intake, double transfer, double shoot) {
        intakeTransfer.moveIntake(intake);
        intakeTransfer.moveTransfer(transfer);
        shooter.setTargetVelocity(shoot);
    }

    public boolean isReadyToShoot() {
        if(
                Math.abs(shooter.getVelocityError()) >= 60 ||
                Math.abs(localization.getAngleToPose(goalPose)) >= 2.5 ||
                Math.abs(localization.getAngularVelocity()) >= 30
        ) {
            return false;
        } else {
            return true;
        }
    }

    // TELEMETRY
    public void telemetryOutput(Telemetry telemetry) {
        long currentTime = System.nanoTime();
        if (lastLoopTime != 0) {
            currentLoopTimeMs = (currentTime - lastLoopTime) / 1_000_000.0;
        }
        lastLoopTime = currentTime;

        telemetry.addLine("--- STATUS ---");
        telemetry.addData("Loop Time", "%.2f ms", currentLoopTimeMs);
        telemetry.addData("Frequency", "%.1f Hz", 1000.0 / currentLoopTimeMs);
        telemetry.addData("Alliance", DataPasser.currentAlliance);
        telemetry.addData("Action Timer", "%.1f ms", DataPasser.getElapsed() * 1000);

        telemetry.addLine("--- LOCALIZATION ---");
        telemetry.addLine(localization.getTelemetry());

        telemetry.addLine("--- DRIVETRAIN ---");
        telemetry.addLine(drivetrain.getTelemetry());

        telemetry.addLine("--- SCORING ---");
        telemetry.addData("Shooter", shooter.getTelemetry());
        telemetry.addData("Intake & Transfer", intakeTransfer.getTelemetry());
        telemetry.addData("Dist to Goal", "%.2f in", localization.getDistanceToPose(goalPose));
    }
}