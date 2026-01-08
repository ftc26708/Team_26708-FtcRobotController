package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Decode Autonomous")
public class DecodeAutonomous extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private final double SHOOTER_VELOCITY = 1500;
    private Pose startPose;
    private Pose shootPose;
    private Pose controlPose;
    private Pose startPickupPose;
    private Pose endPickupPose;
    private Pose finalPose;
    private PathChain scorePreload, grabPickup, scorePickup, scoreEnd;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    @Override
    public void init() {
        telemetry.addLine("Left bumper: Blue");
        telemetry.addLine("Right bumper: Red");
        telemetry.update();

        while (!gamepad1.left_bumper && !gamepad1.right_bumper) {}

        double alliance;
        if (gamepad1.left_bumper) {
            alliance = 1;
        } else {
            alliance = -1;
        }

        telemetry.addLine("Left bumper: Near");
        telemetry.addLine("Right bumper: Far");
        telemetry.update();

        while (!gamepad1.left_bumper && !gamepad1.right_bumper) {}

        if (gamepad1.left_bumper) {
            startPose = new Pose(72 - (72 - 25.28) * alliance, 132.75, Math.toRadians(90 + 54.046 * alliance));
        } else {
            startPose = new Pose(72 - (72 - 48) * alliance, 8.19, Math.toRadians(90 + 0 * alliance));
        }

        shootPose = new Pose(72 - (72 - 48) * alliance, 96, Math.toRadians(90 + 45 * alliance));
        controlPose = new Pose(72 - (72 - 60) * alliance, 84);
        startPickupPose = new Pose(72 - (72 - 48) * alliance, 84, Math.toRadians(90 - 90 * alliance));
        endPickupPose = new Pose(72 - (72 - 18) * alliance, 84, Math.toRadians(90 - 90 * alliance));
        finalPose = new Pose(72 - (72 - 48) * alliance, 60, Math.toRadians(90 - 90 * alliance));

        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IN");
        transferMotor = hardwareMap.get(DcMotorEx.class, "TR");

        shooters = new DcMotorEx[]{leftShooter, rightShooter};

        PIDFCoefficients coeffs = new PIDFCoefficients(15.0, 0.2, 2.5, 13.2);
        for (DcMotorEx motor : shooters) {
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
        transferMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPose, startPickupPose))
                .addPath(new BezierLine(startPickupPose, endPickupPose))
                .setGlobalConstantHeadingInterpolation(endPickupPose.getHeading())
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(endPickupPose, shootPose))
                .setLinearHeadingInterpolation(endPickupPose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        scoreEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading(), 0.75)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                for (DcMotorEx motor : shooters) {
                    motor.setVelocity(SHOOTER_VELOCITY * 7/15);
                }
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2.3) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 4) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 5) {
                    break;
                }

                transferMotor.setPower(0);
                follower.followPath(grabPickup, 0.4, true);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                intakeMotor.setPower(0);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2.3) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 4) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 5) {
                    break;
                }

                transferMotor.setPower(0);
                intakeMotor.setPower(0);
                for (DcMotorEx motor : shooters) {
                    motor.setVelocity(0);
                }
                follower.followPath(scoreEnd, true);
                setPathState(6);
                break;

            case 6:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}