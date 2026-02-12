package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "DECODE TeleOp")
public class DecodeTeleOp extends LinearOpMode {
    // Motors
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;

    // Sensors
    private Limelight3A limelight;
    private IMU imu;

    // Constants
    private double targetX;
    private double targetY;
    private final double KP = 0.075;
    private final double KF = 0.015;
    private final double MAX_TURN_OUTPUT = 0.75;

    // Variables
    private double horizontalDistance = -1;
    private double yaw = 0;
    private boolean autoAimEnabled = false;
    private double targetShooterVelocity = 0;
    private double aimedShooterSpeed = 0;

    // AUTO SEQUENCE STATE
    private boolean autoSequenceActive = false;
    private long autoSequenceStartTime = 0;

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

        while (opModeIsActive()) {
            limelightLogic();
            drive();
            intakeAndTransfer();
            shooterLogic();
            updateTelemetry();
        }
    }

    private void limelightLogic() {
        // Autonomous must be initialized while facing opposite alliance wall for auto-aiming to work
        targetX = -1.4827; // April tag position
        if (DataPasser.currentAlliance == DataPasser.Alliance.RED) {
            // Adjusts yaw sent to Limelight as its coordinate system is different
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 90;
            targetY = 1.4133; // Red goal
        } else {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;
            targetY = -1.4133; // Blue goal
        }

        // Normalizes yaw values
        while (yaw > 180)   yaw -= 360;
        while (yaw <= -180) yaw += 360;

        // Updates Limelight values
        limelight.updateRobotOrientation(yaw);
        LLResult result = limelight.getLatestResult();

        // If result valid, calculates position of bot and distance to goal
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double dx = targetX - botpose.getPosition().x;
                double dy = targetY - botpose.getPosition().y;
                horizontalDistance = Math.sqrt(dx * dx + dy * dy);
            }
        } else {
            horizontalDistance = -1; // Placeholder value as no valid result
        }

        // Redundancy in case of Autonomous program failing
        if (gamepad1.dpad_left) {
            DataPasser.currentAlliance = DataPasser.Alliance.BLUE;
            imu.resetYaw();
            limelight.pipelineSwitch(0);
        }
        if (gamepad1.dpad_right) {
            DataPasser.currentAlliance = DataPasser.Alliance.RED;
            imu.resetYaw();
            limelight.pipelineSwitch(1);
        }

        // Disables chassis auto-aim if gamepad 1 touches sticks
        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
            autoAimEnabled = false;
        }

        // Overrides above case
        if (gamepad2.right_trigger_pressed) {
            autoAimEnabled = true;
        }
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        LLResult result = limelight.getLatestResult();

        if (autoAimEnabled && result != null && result.isValid()) {
            double tx = result.getTx();
            turn = tx * KP + Math.signum(tx) * KF;
            turn = Math.max(-MAX_TURN_OUTPUT, Math.min(MAX_TURN_OUTPUT, turn));
        }

        double LB = drive + turn - strafe;
        double RB = drive - turn + strafe;
        double LF = drive + turn + strafe;
        double RF = drive - turn - strafe;

        double coeff = (gamepad1.left_stick_button || gamepad1.right_stick_button) ? 2800.0 : 1200.0;

        leftBackDrive.setVelocity(LB * coeff);
        rightBackDrive.setVelocity(RB * coeff);
        leftFrontDrive.setVelocity(LF * coeff);
        rightFrontDrive.setVelocity(RF * coeff);
    }

    private void intakeAndTransfer() {
        if (!autoSequenceActive) {
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
        // 1. Manual Overrides (Buttons)
        if (gamepad2.a && !autoSequenceActive) {
            // Start Auto Sequence if A is pressed
            autoSequenceActive = true;
            autoSequenceStartTime = System.currentTimeMillis();
        }
        
        if (gamepad2.b)  shooterMode = ShooterMode.CLOSE;
        if (gamepad2.x)  shooterMode = ShooterMode.MID;
        if (gamepad2.y)  shooterMode = ShooterMode.FAR;
        if (gamepad2.right_stick_button) shooterMode = ShooterMode.BACK;
        if (gamepad2.right_bumper) shooterMode = ShooterMode.AUTO;
        if (gamepad2.dpad_down) shooterMode = ShooterMode.ZERO;

        // 2. Auto Sequence Execution
        if (autoSequenceActive) {
            autoAimEnabled = true;
            shooterMode = ShooterMode.AUTO;
            intakeMotor.setPower(-1.0);
            transferMotor.setPower(-1.0);

            if (System.currentTimeMillis() - autoSequenceStartTime >= 4000) {
                autoSequenceActive = false;
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                autoAimEnabled = false;
            }
        }

        // 3. Calculation Logic
        if (horizontalDistance > 0 && shooterMode == ShooterMode.AUTO) {
            aimedShooterSpeed = (0.00333057 * Math.pow(33.55221, horizontalDistance)) +
                    (191.64674 * horizontalDistance) +
                    971.0142;
        }

        // 4. Determine final velocity based on mode
        switch (shooterMode) {
            case AUTO:
                targetShooterVelocity = (horizontalDistance > 0) ? aimedShooterSpeed : 0;
                break;
            case CLOSE: targetShooterVelocity = 1200; break;
            case MID:   targetShooterVelocity = 1350; break;
            case FAR:   targetShooterVelocity = 1700; break;
            case BACK:  targetShooterVelocity = -500; break;
            case ZERO:
            default:    targetShooterVelocity = 0;    break;
        }

        // 5. Safety Clamp and Application
        targetShooterVelocity = Math.min(targetShooterVelocity, 2000);
        double scaledVelocity = targetShooterVelocity * 7.0 / 15.0;

        leftShooter.setVelocity(scaledVelocity);
        rightShooter.setVelocity(scaledVelocity);
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

        PIDFCoefficients coeffs = new PIDFCoefficients(45.0, 0.02, 2.5, 13.2);
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

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "LM");
        limelight.pipelineSwitch(DataPasser.currentAlliance == DataPasser.Alliance.RED ? 1 : 0);
        limelight.start();
    }

    private void updateTelemetry() {
        telemetry.addData("Target Found", (horizontalDistance == -1) ? "NO" : "YES");
        telemetry.addData("Distance to Target", "%.2f", horizontalDistance);
        telemetry.addData("Shooter Velocity", targetShooterVelocity);
        telemetry.addData("Actual Left Velocity", leftShooter.getVelocity() * 15.0 / 7.0);
        telemetry.addData("Actual Right Velocity", rightShooter.getVelocity() * 15.0 / 7.0);
        telemetry.addData("Auto-Aim Active", autoAimEnabled);
        telemetry.addLine("Drive Mode: " + ((gamepad1.left_stick_button || gamepad1.right_stick_button) ? "FAST (2800)" : "SLOW (1200)"));
        telemetry.update();
    }
}
