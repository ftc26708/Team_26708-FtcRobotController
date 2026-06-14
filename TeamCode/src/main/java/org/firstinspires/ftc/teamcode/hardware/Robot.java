package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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
    private final double AIM_KP = 0.015;
    private final double AIM_KD = 0.0005;
    private final double AIM_KF = 0.035;
    final double AIM_K_GEO = 0.0008;
    private final double MAX_TURN = 0.5;
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
        ShootOnTheMove.SOTMResult sotmResult = ShootOnTheMove.calculateSOTM(
                follower.getPose(),
                follower.getVelocity(),
                goalPose
        );

        // 1. Calculate and normalize the shortest-path heading error
        double headingError = sotmResult.targetHeading - Math.toDegrees(follower.getHeading());
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        // 2. Feedback loop: Calculate the baseline PDF power to correct position error
        double headingAdjustment = (headingError * AIM_KP) +
                (Math.signum(headingError) * AIM_KF) -
                Math.toDegrees(follower.getAngularVelocity()) * AIM_KD;

        // 3. PROACTIVE KINEMATIC GEOMETRIC FEEDFORWARD (The Cross-Product Matrix)
        double proactiveTurnPower = 0.0;

        // Grab the virtual goal positions
        double virtualDist = 0.0254 * sotmResult.virtualGoalPose.distanceFrom(follower.getPose());

        if (virtualDist > 0.1) {
            // Robot tracking coordinates (converted to meters)
            double rX = follower.getPose().getX() * 0.0254;
            double rY = follower.getPose().getY() * 0.0254;

            // Displacement vector to the virtual goal
            double dx = (sotmResult.virtualGoalPose.getX()) - rX;
            double dy = (sotmResult.virtualGoalPose.getY()) - rY;

            // Normalize displacement vector to get unit coordinates pointing to target
            double ux = dx / virtualDist;
            double uy = dy / virtualDist;

            // Robot field-centric velocity vector components (converted to meters/sec)
            double vX = follower.getVelocity().getXComponent() * 0.0254;
            double vY = follower.getVelocity().getYComponent() * 0.0254;

            // 2D Cross-Product / Determinant to isolate tangential velocity component
            double tangentialVelocity = (vX * uy) - (vY * ux);

            // Required angular tracking speed scales inversely with proximity (omega = v / r)
            double requiredOmega = tangentialVelocity / virtualDist;

            // Bridge kinematics to direct motor command scale
            proactiveTurnPower = AIM_K_GEO * requiredOmega;
        }

        // 4. Combine Feedback + Predictive Feedforward (Cross-Product Matrix)
        // The feedforward naturally biases the power command in the direction of the sweep
        double totalTurnPower = headingAdjustment + proactiveTurnPower;

        // 5. Clamp the final combined adjustment
        double clampedTurn = Math.max(-MAX_TURN, Math.min(MAX_TURN, totalTurnPower));

        // 6. Pass to drivetrain
        drivetrain.drive(x, y, clampedTurn);
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
    public void intake(double intakeSpeed, double transferSpeed) {
        intakeTransfer.moveIntake(intakeSpeed);
        intakeTransfer.moveTransfer(transferSpeed);
        shooter.setTargetVelocity(-1200);
    }
    public void prepareSpinUp() {
        intakeTransfer.positionIntake(0);
        intakeTransfer.positionTransfer(-0.15);
        shooter.setTargetVelocity(-1200);
    }
    public void spinUp() {
        intakeTransfer.positionIntake(0);
        intakeTransfer.positionTransfer(-0.15);
        shooter.setTargetVelocity(getNeededRPM());
    }
    public void shoot() {
        intakeTransfer.moveIntake(1);
        intakeTransfer.moveTransfer(1);
        shooter.setTargetVelocity(getNeededRPM());
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
    public double getNeededRPM() {
        return ShootOnTheMove.calculateSOTM(
                follower.getPose(),
                follower.getVelocity(),
                goalPose).targetRPM;
    }

    public static class ShootOnTheMove {

        public static SOTMResult calculateSOTM(Pose robotPose, Vector robotVel, Pose goalPose) {
            // Convert the dynamic goal coordinates from inches to meters
            double goalXMeters = goalPose.getX() * 0.0254;
            double goalYMeters = goalPose.getY() * 0.0254;

            // Convert localized tracking positions from inches to meters
            double rX = robotPose.getX() * 0.0254;
            double rY = robotPose.getY() * 0.0254;

            // Convert translational field-centric velocities from inches/sec to meters/sec
            double vX = robotVel.getXComponent() * 0.0254;
            double vY = robotVel.getYComponent() * 0.0254;

            // Establish real baseline geometric distance to the chosen goal
            double realDist = Math.hypot(goalXMeters - rX, goalYMeters - rY);

            // Pre-calculate launch angle cosine constant to save CPU cycles
            final double COS_LAUNCH_ANGLE = Math.cos(Math.toRadians(55.86));

            // Dynamic Seed: Estimate initial TOF using the actual baseline distance
            double initialVelocity = TrajectoryCalculator.distanceToVelocity(Math.max(1.25, Math.min(4.0, realDist)));
            double estimatedTOF = realDist / (initialVelocity * COS_LAUNCH_ANGLE);

            double virtualX = goalXMeters;
            double virtualY = goalYMeters;
            double virtualDist = realDist;
            double requiredVelocity = 0.0;

            // 3-Pass Convergence Machine
            for (int i = 0; i < 3; i++) {
                // Shift the target point completely opposite to the robot's translation vector
                virtualX = goalXMeters - (vX * estimatedTOF);
                virtualY = goalYMeters - (vY * estimatedTOF);

                // Re-evaluate the virtual geometric distance to this fake point
                virtualDist = Math.hypot(virtualX - rX, virtualY - rY);

                // Clip the distance to possible shooter constraints
                double clampedDist = Math.max(1.25, Math.min(4.0, virtualDist));

                // Execute your piecewise ballistic equation solver
                requiredVelocity = TrajectoryCalculator.distanceToVelocity(clampedDist);

                // Extract the horizontal speed component to tighten our flight time projection
                double horizontalVel = requiredVelocity * COS_LAUNCH_ANGLE;

                if (horizontalVel > 0) {
                    // Time of Flight = Distance / Horizontal Velocity
                    // Note: Using un-clamped virtualDist ensures vector math stays geometrically sound
                    estimatedTOF = virtualDist / horizontalVel;
                }
            }

            // Update virtual goal pose in main class
            Pose virtualGoalPose = new Pose(virtualX / 0.0254, virtualY / 0.0254);

            // Calculate absolute field-centric heading angle the robot must face (degrees)
            double targetHeading = Math.toDegrees(Math.atan2(virtualY - rY, virtualX - rX));

            // Convert kinematic launch velocity to physical motor speed target using your logarithmic model
            double targetRPM = TrajectoryCalculator.convertVelocityToRpm(requiredVelocity);

            // Bundle it up and return the payload!
            return new SOTMResult(targetHeading, targetRPM, virtualGoalPose);
        }

        // OUTPUT CONTAINER CLASS
        public static class SOTMResult {
            public final double targetHeading; // Absolute field-centric heading in degrees
            public final double targetRPM;     // Calculated shooter wheel velocity in RPM
            public final Pose virtualGoalPose; // Virtual goal pose in field-centric coordinates

            public SOTMResult(double targetHeading, double targetRPM, Pose virtualGoalPose) {
                this.targetHeading = targetHeading;
                this.targetRPM = targetRPM;
                this.virtualGoalPose = virtualGoalPose;
            }
        }

        // TRAJECTORY CALCULATOR CARRIER
        public static class TrajectoryCalculator {
            // Raw input constants
            private static final double G = 9.81;
            private static final double THETA_L_DEG = 55.86;
            private static final double THETA_L_RAD = Math.toRadians(THETA_L_DEG);

            private static final double X1 = 0.16512 - (0.0635 * Math.sin(THETA_L_RAD));
            private static final double Y1 = 0.33533 + (0.0635 * Math.cos(THETA_L_RAD));

            private static final double I1 = 0.25;
            private static final double I2 = 0.19;
            private static final double M1 = 1.22;
            private static final double M2 = 1.72;

            private static final double TAN_THETA = Math.tan(THETA_L_RAD);
            private static final double COEF_V = Math.sqrt(G / (2.0 * Math.pow(Math.cos(THETA_L_RAD), 2)));
            private static final double RAMP_SLOPE = (I2 - I1) / (M2 - M1);

            private static final double LN_RPM_BASE = Math.log(0.999257);

            /**
             * Calculates the required launch velocity V(x) given a distance in meters x.
             * Valid domain: 1.25 <= x <= 4.0
             */
            public static double distanceToVelocity(double x) {
                if (x < 1.25 || x > 4.0) {
                    throw new IllegalArgumentException("Position x must be between 1.25 and 4.0");
                }

                // D is the height above the ground that the robot aims at given a distance x
                double d;
                if (x <= M2) {
                    d = I1 + RAMP_SLOPE * (x - M1);
                } else {
                    d = I2;
                }

                // Final calculation
                double numerator = COEF_V * (x - X1);
                double denominator = Math.sqrt((TAN_THETA * x) - (TAN_THETA * X1) + Y1 - 0.98425 - d);

                return numerator / denominator;
            }

            /**
             * Calculates the required RPM of the shooter to achieve the inputted velocity.
             * Valid domain: 0.0 <= x < 8.0
             */
            public static double convertVelocityToRpm(double velocity) {
                if (velocity >= 8.0) {
                    throw new IllegalArgumentException("Maximum possible velocity is 8 m/s.");
                }
                if (velocity < 0) {
                    throw new IllegalArgumentException("Miminum possible velocity is 0 m/s.");
                }

                // Empirical Desmos-fitted equation
                double numeratorLogTerm = -(velocity - 9.32042) / 10.11417;
                return Math.log(numeratorLogTerm) / LN_RPM_BASE;
            }
        }
    }

    public boolean isReadyToShoot() {
        return !(Math.abs(shooter.getVelocityError()) >= 60) &&
                !(Math.abs(localization.getAngleToPose(ShootOnTheMove.calculateSOTM(follower.getPose(), follower.getVelocity(), goalPose).virtualGoalPose)) >= 2);
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
        telemetry.addData("Heading Error", localization.getAngleToPose(ShootOnTheMove.calculateSOTM(follower.getPose(), follower.getVelocity(), goalPose).virtualGoalPose));

        telemetry.addLine("--- DRIVETRAIN ---");
        telemetry.addLine(drivetrain.getTelemetry());

        telemetry.addLine("--- SCORING ---");
        telemetry.addData("Shooter", shooter.getTelemetry());
        telemetry.addData("Intake & Transfer", intakeTransfer.getTelemetry());
        telemetry.addData("Dist to Goal", "%.2f in", localization.getDistanceToPose(goalPose));
    }
}