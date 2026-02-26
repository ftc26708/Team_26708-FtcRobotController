package org.firstinspires.ftc.teamcode;

// Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "DECODE TeleOp")
public class DecodeTeleOp extends LinearOpMode {
    // Motors
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;

    // Sensors
    private Limelight3A limelight;

    // Constants
    final double MOUNT_HEIGHT = 0.26035; // Meters
    final double TARGET_HEIGHT = 0.74930; // Meters
    final double MOUNT_ANGLE = 16.0;
    private final double METERS_TO_INCHES = 39.3701;
    private final Pose BLUE_GOAL_POSE = new Pose(6, 138, 0);
    private final Pose RED_GOAL_POSE = new Pose(138, 138, 0);
    private final double KP = 0.020;
    private final double KF = 0.015;
    private final double MAX_TURN_OUTPUT = 0.6;

    // Variables
    private double horizontalDistance = -1;
    private boolean autoAimEnabled = false;
    private double targetShooterVelocity = 0;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean autoParking = false;
    private Follower follower; // This handles the PedroPathing movement
    private Pose currentPose = DataPasser.endAutoPose;
    private boolean drivetrainReady = false;
    private boolean shooterReady = false;
    private double aimedShooterSpeed = 0;
    private boolean autoAimedLastFrame = false;
    private boolean relocalizedThisCycle = false;
    private double odometryTurnError = 0;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private enum DriveSpeed {
        SLOW,
        FAST,
        ULTRA
    }
    private DriveSpeed driveSpeed = DriveSpeed.SLOW;
    private enum ShooterMode {
        AUTO,
        BACK,
        ZERO,
        CLOSE,
        MID,
        FAR
    }

    ShooterMode shooterMode = ShooterMode.AUTO;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "Initialized, waiting");
        telemetry.update();
        waitForStart();
        matchTimer.reset();

        while (opModeIsActive()) {
            if (matchTimer.seconds() > 120) {
                stop();
            } else {
                localization();
                drive();
                intakeAndTransfer();
                shooterLogic();
                updateTelemetry();
            }
        }
    }

    private void localization() {
        follower.update();
        currentPose = follower.getPose();

        // Nudges the heading by 0.5 degrees per distinct press
        // D-pad Left increases angle (Counter-Clockwise), Right decreases (Clockwise)
        if (gamepad2.dpad_left && !lastDpadLeft) {
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + Math.toRadians(0.5)));
        }
        if (gamepad2.dpad_right && !lastDpadRight) {
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() - Math.toRadians(0.5)));
        }
        // Reset yaw
        if (gamepad2.dpad_up) {
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(90)));

        }

        lastDpadLeft = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;

        if (gamepad2.right_trigger > 0.5 && !autoAimedLastFrame) {
            autoAimEnabled = true;
            relocalizedThisCycle = false;
        }
        autoAimedLastFrame = gamepad2.right_trigger > 0.5;

        if (autoAimEnabled && !relocalizedThisCycle) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double ty = result.getTy();
                double tx = result.getTx();
                double totalAngleRadians = Math.toRadians(MOUNT_ANGLE + ty);
                double visionDistanceMeters = (TARGET_HEIGHT - MOUNT_HEIGHT) / Math.tan(totalAngleRadians);
                double visionDistanceInches = visionDistanceMeters * METERS_TO_INCHES;

                Pose target = (DataPasser.currentAlliance == DataPasser.Alliance.RED) ? RED_GOAL_POSE : BLUE_GOAL_POSE;
                double angleToGoalField = currentPose.getHeading() - Math.toRadians(tx);

                double newX = target.getX() - (Math.cos(angleToGoalField) * visionDistanceInches);
                double newY = target.getY() - (Math.sin(angleToGoalField) * visionDistanceInches);

                follower.setPose(new Pose(newX, newY, currentPose.getHeading()));
                relocalizedThisCycle = true;
            }
        }

        Pose target = (DataPasser.currentAlliance == DataPasser.Alliance.RED) ? RED_GOAL_POSE : BLUE_GOAL_POSE;
        double dx = target.getX() - currentPose.getX();
        double dy = target.getY() - currentPose.getY();
        horizontalDistance = Math.hypot(dx, dy);

        double targetAngle = Math.atan2(dy, dx);
        double relativeAngle = targetAngle - currentPose.getHeading();
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
        odometryTurnError = Math.toDegrees(relativeAngle);

        if (gamepad1.dpad_left) {
            DataPasser.currentAlliance = DataPasser.Alliance.BLUE;
            limelight.pipelineSwitch(0);
        }
        if (gamepad1.dpad_right) {
            DataPasser.currentAlliance = DataPasser.Alliance.RED;
            limelight.pipelineSwitch(1);
        }

        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
            autoAimEnabled = false;
        }
    }

    private void drive() {
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            driveSpeed = DriveSpeed.ULTRA;
        } else if (gamepad1.a) {
            driveSpeed = DriveSpeed.SLOW;
        } else {
            driveSpeed = DriveSpeed.FAST;
        }

        double speedMultiplier;
        switch (driveSpeed) {
            case ULTRA: speedMultiplier = 1.0;  break;
            case FAST:  speedMultiplier = 0.40; break;
            case SLOW:
            default:    speedMultiplier = 0.20; break;
        }

        double forwardRaw = -gamepad1.left_stick_y * speedMultiplier;
        double strafeLeftRaw = -gamepad1.left_stick_x * speedMultiplier;
        double turnRaw = -gamepad1.right_stick_x * speedMultiplier;

        drivetrainReady = false;
        if (autoAimEnabled) {
            if (Math.abs(odometryTurnError) < 2.5) {
                turnRaw = 0;
                drivetrainReady = true;
            } else {
                turnRaw = (odometryTurnError * KP) + (Math.signum(odometryTurnError) * KF);
                turnRaw = Math.max(-MAX_TURN_OUTPUT, Math.min(MAX_TURN_OUTPUT, turnRaw));
            }
        }

        if (DataPasser.currentAlliance == DataPasser.Alliance.RED) {
            follower.setTeleOpDrive(-forwardRaw, -strafeLeftRaw, turnRaw, false);
        } else {
            follower.setTeleOpDrive(forwardRaw, strafeLeftRaw, turnRaw, false);
        }
    }

    private void intakeAndTransfer() {
        // AUTO-FIRE SEQUENCE
        if (drivetrainReady && shooterReady && autoAimEnabled) {
            // Fire only when fully aimed
            intakeMotor.setPower(0.5); // Spin intake forward
            transferMotor.setPower(1); // Spin transfer to shoot
        } else {
            // This runs only if the auto-fire sequence isn't active
            intakeMotor.setPower(-gamepad2.left_stick_y);
            if (gamepad2.left_bumper) {
                transferMotor.setPower(-1.0);
            } else {
                transferMotor.setPower(0.6 * gamepad2.left_trigger - 0.2);
            }
        }
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }


    private void shooterLogic() {
        // 1. Manual Mode Setting
        if      (gamepad2.a)                   shooterMode = ShooterMode.BACK;
        else if (gamepad2.b)                   shooterMode = ShooterMode.CLOSE;
        else if (gamepad2.x)                   shooterMode = ShooterMode.MID;
        else if (gamepad2.y)                   shooterMode = ShooterMode.FAR;
        else if (gamepad2.right_trigger > 0.5) shooterMode = ShooterMode.AUTO;
        else                                   shooterMode = ShooterMode.ZERO;

        // 2. Find necessary velocity
        if (horizontalDistance > 0) {
            // Tuning Constants
            double A = 25.5 / Math.pow(METERS_TO_INCHES, 2);
            double B = 143.0 / METERS_TO_INCHES;
            double C = 980.0;

            aimedShooterSpeed = ((A * Math.pow(horizontalDistance, 2)) + (B * horizontalDistance) + C);
        }

        // 3. Determine final velocity based on mode
        switch (shooterMode) {
            case AUTO:
                targetShooterVelocity = (horizontalDistance > 0) ? aimedShooterSpeed : 0;
                break;
            case CLOSE: targetShooterVelocity = 1200; break;
            case MID:   targetShooterVelocity = 1350; break;
            case FAR:   targetShooterVelocity = 1700; break;
            case BACK:  targetShooterVelocity = -600; break;
            default:    targetShooterVelocity = 0;    break;
        }

        // 4. Clamp Velocity and convert RPM to TPS
        targetShooterVelocity = Math.min(targetShooterVelocity, 2100);
        double scaledVelocity = targetShooterVelocity * 7.0 / 15.0;

        // 5. Set velocities
        leftShooter.setVelocity(scaledVelocity);
        rightShooter.setVelocity(scaledVelocity);

        shooterReady = Math.abs(leftShooter.getVelocity() - scaledVelocity) <= 0.03 * scaledVelocity &&
                Math.abs(rightShooter.getVelocity() - scaledVelocity) <= 0.03 * scaledVelocity;
    }

    private void initHardware() {
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "LB");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "LF");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "RB");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "RF");

        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IN");
        transferMotor = hardwareMap.get(DcMotorEx.class, "TR");

        DcMotor[] drives = {leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive};
        for (DcMotor motor : drives) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotorEx[] loaders = {intakeMotor, transferMotor};
        for (DcMotorEx motor : loaders) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        PIDFCoefficients coeffs = new PIDFCoefficients(90.0, 0.02, 2.5, 13.2);
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx motor : shooters) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        }

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(DataPasser.endAutoPose);
        follower.startTeleopDrive();

        limelight = hardwareMap.get(Limelight3A.class, "LM");
        if (DataPasser.currentAlliance == DataPasser.Alliance.RED) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
        }
        limelight.start();
    }

    private void updateTelemetry() {
        telemetry.addLine("FIELD POSITION");
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Alliance", DataPasser.currentAlliance);
        telemetry.addData("Distance to Goal", "%.2f in", horizontalDistance);

        // --- VISION & STATUS ---
        telemetry.addLine("SYSTEM STATUS");
        telemetry.addData("Limelight Lock", (horizontalDistance != -1) ? "LOCKED" : "SEARCHING");
        telemetry.addData("Relocalized", relocalizedThisCycle ? "YES" : "NO");
        telemetry.addData("Auto-Aim Active", autoAimEnabled);

        telemetry.addLine("MECHANISMS");
        telemetry.addData("Mode", shooterMode);
        telemetry.addData("Ready to Fire", (drivetrainReady && shooterReady) ? "READY" : "WAITING");
        telemetry.addData("Target RPM", targetShooterVelocity);
        telemetry.addData("Actual L/R RPM", "%.0f / %.0f",
                leftShooter.getVelocity() * 15/7, rightShooter.getVelocity() * 15/7);

        String modeName = driveSpeed.toString();
        double powerPercent = (driveSpeed == DriveSpeed.ULTRA) ? 100 : (driveSpeed == DriveSpeed.FAST ? 40 : 20);
        telemetry.addData("Drive", "%s (%.0f%%)", modeName, powerPercent);

        telemetry.update();
    }
}