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

@Autonomous(name = "DECODE Autonomous")
public class DecodeAutonomous12 extends OpMode {
    private enum InitState {
        HARDWARE,
        SELECT_ALLIANCE,
        SELECT_SIDE,
        COMPUTE_POSES,
        BUILD_PATHS,
        READY
    }
    private InitState initState = InitState.HARDWARE;
    private double alliance = 1;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private final double SHOOTER_VELOCITY = 1500;
    private Pose startPose;
    private Pose shootPose;
    private Pose controlPose1;
    private Pose endPickupPose1;
    private Pose controlGatePose;
    private Pose openGatePose;
    private Pose firstControlPose2;
    private Pose secondControlPose2;
    private Pose endPickupPose2;
    private Pose controlPose3;
    private Pose endPickupPose3;
    private Pose finalPose;
    private PathChain scorePreload, grabPickup1, scorePickup1, openGate, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    public void init() {}

    @Override
    public void init_loop() {
        switch (initState) {
            case HARDWARE:
                telemetry.addLine("Initializing hardware...");

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

                follower = Constants.createFollower(hardwareMap);
                pathTimer = new Timer();

                initState = InitState.SELECT_ALLIANCE;
                break;

            case SELECT_ALLIANCE:
                telemetry.addLine("Select Alliance (GAMEPAD 1):");
                telemetry.addLine("Left bumper = BLUE");
                telemetry.addLine("Right bumper = RED");

                if (gamepad1.left_bumper) {
                    alliance = 1;
                    initState = InitState.SELECT_SIDE;
                }
                if (gamepad1.right_bumper) {
                    alliance = -1;
                    initState = InitState.COMPUTE_POSES;
                }
                break;

            case COMPUTE_POSES:
                // Will move to nearSide = true block after far added
                startPose = new Pose(72 - (72 - 15.78) * alliance, 123.52, Math.toRadians(90 + 90 * alliance));
                shootPose = new Pose(72 - (72 - 48) * alliance, 96, Math.toRadians(90 + 45 * alliance));
                controlPose1 = new Pose(72 - (72 - 48) * alliance, 84);
                endPickupPose1 = new Pose(72 - (72 - 18) * alliance, 84, Math.toRadians(90 - 90 * alliance));
                controlGatePose = new Pose(72 - (72 - 24) * alliance, 72);
                openGatePose = new Pose(72 - (72 - 16) * alliance, 72, Math.toRadians(90 - 90 * alliance));
                firstControlPose2 = new Pose(72 - (72 - 48) * alliance, 60);
                secondControlPose2 = new Pose(72 - (72 - 36) * alliance, 60);
                endPickupPose2 = new Pose(72 - (72 - 18) * alliance, 60, Math.toRadians(90 - 90 * alliance));
                controlPose3 = new Pose(72 - (72 - 48) * alliance, 36);
                endPickupPose3 = new Pose(72 - (72 - 18) * alliance, 36, Math.toRadians(90 - 90 * alliance));
                finalPose = new Pose(72 - (72 - 57.5) * alliance, 108, Math.toRadians(90 + 58 * alliance));

                initState = InitState.BUILD_PATHS;
                break;

            case BUILD_PATHS:
                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup1 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, controlPose1, endPickupPose1))
                        .setReversed()
                        .build();

                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierLine(endPickupPose1, shootPose))
                        .setLinearHeadingInterpolation(endPickupPose1.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup2 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, firstControlPose2, secondControlPose2, endPickupPose2))
                        .setReversed()
                        .build();

                openGate = follower.pathBuilder()
                        .addPath(new BezierCurve(endPickupPose2, controlGatePose, openGatePose))
                        .setReversed()
                        .build();

                scorePickup2 = follower.pathBuilder()
                        .addPath(new BezierLine(endPickupPose2, shootPose))
                        .setLinearHeadingInterpolation(endPickupPose2.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup3 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, controlPose3, controlPose3, endPickupPose3))
                        .setReversed()
                        .build();

                scorePickup3 = follower.pathBuilder()
                        .addPath(new BezierLine(endPickupPose3, finalPose))
                        .setLinearHeadingInterpolation(endPickupPose3.getHeading(), finalPose.getHeading(), 0.75)
                        .build();

                follower.setStartingPose(startPose);
                initState = InitState.READY;
                break;

            case READY:
                telemetry.addLine("READY!");
                break;
        }

        telemetry.addData("Alliance", alliance == 1 ? "BLUE" : "RED");
        telemetry.update();
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
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.35) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.35) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3.1) {
                    break;
                }

                transferMotor.setPower(0);
                follower.followPath(grabPickup1, true);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
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
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.5) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3.25) {
                    break;
                }

                transferMotor.setPower(0);
                follower.followPath(grabPickup2, true);
                setPathState(6);
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(openGate, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;

            case 9:
                intakeMotor.setPower(0);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.5) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3.25) {
                    break;
                }

                transferMotor.setPower(0);
                follower.followPath(grabPickup3, true);
                setPathState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12:
                intakeMotor.setPower(0);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.5) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3.25) {
                    break;
                }

                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                setPathState(13);
                break;

            case 13:
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}