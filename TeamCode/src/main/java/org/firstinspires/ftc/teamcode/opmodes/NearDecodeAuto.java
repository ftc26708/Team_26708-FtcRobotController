package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "DECODE Autonomous (Near)", preselectTeleOp = "DECODE TeleOp")
public class NearDecodeAuto extends BaseDecodeAuto {
    public int numberOfGateCycles = 0;
    public long startTime;
    public double timeTaken;
    public enum PathState {
        INITIAL,
        MOVE_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        PICKUP_MIDDLE_SPIKE,
        MOVE_TO_SHOOT_MIDDLE_SPIKE,
        SHOOT_MIDDLE_SPIKE,
        MOVE_TO_OPEN_GATE,
        WAIT_AND_OPEN_GATE,
        MOVE_TO_INTAKE_GATE,
        WAIT_AND_INTAKE_GATE,
        MOVE_TO_SHOOT_GATE,
        SHOOT_GATE,
        PICKUP_CLOSE_SPIKE,
        MOVE_TO_SHOOT_CLOSE_SPIKE,
        SHOOT_CLOSE_SPIKE,
        FINISHED
    }

    private Pose
            firstShootPose, shootPose, finalShootPose,
            middleSpikeControlPose1, middleSpikeControlPose2, middleSpikeEndPose,
            midwayShootControl, gatePose, gateIntakePose,
            closeSpikeControlPose, closeSpikeEndPose;

    private PathChain
            scorePreload,
            pickupMiddleSpike, scoreMiddleSpike,
            goToGate, intakeGate, scoreGate,
            pickupCloseSpike, scoreCloseSpike;

    @Override
    protected Pose getStartCalibrationPose() {
        return new Pose(38.76, 133.31, Math.toRadians(90));
    }

    @Override
    protected void computePoses() {
        // Blue alliance base coordinates mirrored via alliancePose()
        firstShootPose = alliancePose(new Pose(35, 96.5, Math.toRadians(127.875)));
        shootPose = alliancePose(new Pose(50, 81.5, Math.toRadians(129.805)));
        finalShootPose = alliancePose(new Pose(48, 110.75, Math.toRadians(147.355)));

        middleSpikeControlPose1 = alliancePose(new Pose(49, 76));
        middleSpikeControlPose2 = alliancePose(new Pose(60, 58));
        middleSpikeEndPose = alliancePose(new Pose(11, 58, Math.toRadians(0)));

        midwayShootControl = alliancePose(new Pose(32, 59.5));
        gatePose = alliancePose(new Pose(11.75, 59.5, Math.toRadians(-30)));
        gateIntakePose = alliancePose(new Pose(11.75, 54, Math.toRadians(0)));

        closeSpikeControlPose = alliancePose(new Pose(41, 82.5));
        closeSpikeEndPose = alliancePose(new Pose(17, 82.5, Math.toRadians(0)));
    }

    @Override
    protected void buildPaths(Pose startPose) {
        scorePreload = robot.pathBuilder()
                .addPath(new BezierLine(startPose, firstShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        pickupMiddleSpike = robot.pathBuilder()
                .addPath(new BezierCurve(firstShootPose, middleSpikeControlPose1, middleSpikeControlPose2, middleSpikeEndPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scoreMiddleSpike = robot.pathBuilder()
                .addPath(new BezierCurve(middleSpikeEndPose, midwayShootControl, shootPose))
                .setLinearHeadingInterpolation(middleSpikeEndPose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        goToGate = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, midwayShootControl, gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading(), 0.5)
                .setTValueConstraint(0.97)
                .build();

        intakeGate = robot.pathBuilder()
                .addPath(new BezierLine(gatePose, gateIntakePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateIntakePose.getHeading(), 1, 0.6)
                .build();

        scoreGate = robot.pathBuilder()
                .addPath(new BezierCurve(gateIntakePose, midwayShootControl, shootPose))
                .setLinearHeadingInterpolation(gateIntakePose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        pickupCloseSpike = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, closeSpikeControlPose, closeSpikeEndPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        scoreCloseSpike = robot.pathBuilder()
                .addPath(new BezierLine(closeSpikeEndPose, finalShootPose))
                .setLinearHeadingInterpolation(closeSpikeEndPose.getHeading(), finalShootPose.getHeading(), 0.75)
                .build();
    }

    @Override
    public void start() {
        pathState = PathState.INITIAL;
    }

    @Override
    protected void stateMachine() {
        switch ((PathState) pathState) {
            case INITIAL:
                startTime = System.nanoTime();
                robot.followPath(scorePreload, true);
                robot.spinUp();
                setPathState(PathState.MOVE_TO_SHOOT_PRELOAD);
                break;

            case MOVE_TO_SHOOT_PRELOAD:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.intake(1, 0.5);
                    robot.followPath(pickupMiddleSpike, true);
                    setPathState(PathState.PICKUP_MIDDLE_SPIKE);
                }
                break;

            case PICKUP_MIDDLE_SPIKE:
                robot.intake(1, 0.5);
                if (robot.isNotPathFollowing()) {
                    robot.followPath(scoreMiddleSpike, true);
                    setPathState(PathState.MOVE_TO_SHOOT_MIDDLE_SPIKE);
                }
                break;

            case MOVE_TO_SHOOT_MIDDLE_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.4)) {
                    robot.spinUp();
                } else if (Robot.DataPasser.hasElapsed(0.2)) {
                    robot.prepareSpinUp();
                } else {
                    robot.intake(0, 0);
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_MIDDLE_SPIKE);
                }
                break;

            case SHOOT_MIDDLE_SPIKE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.followPath(goToGate, true);
                    setPathState(PathState.MOVE_TO_OPEN_GATE);
                }
                break;

            case MOVE_TO_OPEN_GATE:
                robot.intake(1, 0.5);
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_AND_OPEN_GATE);
                }
                break;

            case WAIT_AND_OPEN_GATE:
                robot.intake(1, 0.5);
                if (Robot.DataPasser.hasElapsed(1.2)) {
                    robot.followPath(intakeGate, true);
                    setPathState(PathState.MOVE_TO_INTAKE_GATE);
                }
                break;

            case MOVE_TO_INTAKE_GATE:
                robot.intake(1, 0.5);
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_AND_INTAKE_GATE);
                }
                break;

            case WAIT_AND_INTAKE_GATE:
                robot.intake(1, 0.5);
                if (Robot.DataPasser.hasElapsed(0.6)) {
                    robot.followPath(scoreGate, true);
                    setPathState(PathState.MOVE_TO_SHOOT_GATE);
                }
                break;

            case MOVE_TO_SHOOT_GATE:
                if (Robot.DataPasser.hasElapsed(0.4)) {
                    robot.spinUp();
                } else if (Robot.DataPasser.hasElapsed(0.2)) {
                    robot.prepareSpinUp();
                } else {
                    robot.intake(0, 0);
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_GATE);
                }
                break;

            case SHOOT_GATE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.intake(1, 0.5);
                    numberOfGateCycles++;
                    if (numberOfGateCycles >= 3) {
                        robot.followPath(pickupCloseSpike, true);
                        setPathState(PathState.PICKUP_CLOSE_SPIKE);
                    } else {
                        robot.followPath(goToGate, true);
                        setPathState(PathState.MOVE_TO_OPEN_GATE);
                    }
                }
                break;

            case PICKUP_CLOSE_SPIKE:
                robot.intake(1, 0.5);
                if (robot.isNotPathFollowing()) {
                    robot.followPath(scoreCloseSpike, true);
                    setPathState(PathState.MOVE_TO_SHOOT_CLOSE_SPIKE);
                }
                break;

            case MOVE_TO_SHOOT_CLOSE_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.4)) {
                    robot.spinUp();
                } else if (Robot.DataPasser.hasElapsed(0.2)) {
                    robot.prepareSpinUp();
                } else {
                    robot.intake(0, 0);
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_CLOSE_SPIKE);
                }
                break;

            case SHOOT_CLOSE_SPIKE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    timeTaken = (System.nanoTime() - startTime) / 1000000000.0;
                    telemetry.addData("Time Finished", timeTaken);
                    setPathState(PathState.FINISHED);
                }
                break;

            case FINISHED:
                robot.stop();
                telemetry.addData("Time Finished", timeTaken);
                break;
        }
    }
}