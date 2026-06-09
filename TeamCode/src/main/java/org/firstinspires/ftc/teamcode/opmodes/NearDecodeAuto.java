package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "DECODE Autonomous (Near)", preselectTeleOp = "DECODE TeleOp")
public class NearDecodeAuto extends BaseDecodeAuto {

    public enum PathState {
        INITIAL,
        MOVE_PRELOAD,
        SHOOT_PRELOAD,
        GRAB_PICKUP_MIDDLE_SPIKE,
        WAIT_PICKUP_MIDDLE_SPIKE,
        SCORE_PICKUP_MIDDLE_SPIKE,
        SHOOT_PICKUP_MIDDLE_SPIKE,
        GRAB_PICKUP_GATE,
        INTAKE_PICKUP_GATE,
        WAIT_PICKUP_GATE,
        SCORE_PICKUP_GATE,
        SHOOT_PICKUP_GATE,
        GRAB_PICKUP_CLOSE_SPIKE,
        WAIT_PICKUP_CLOSE_SPIKE,
        SCORE_PICKUP_CLOSE_SPIKE,
        SHOOT_PICKUP_CLOSE_SPIKE,
        GRAB_PICKUP_FAR_SPIKE,
        WAIT_PICKUP_FAR_SPIKE,
        SCORE_PICKUP_FAR_SPIKE,
        SHOOT_PICKUP_FAR_SPIKE,
        PARKING
    }

    private Pose
            shootPose,
            pickupMiddleSpikeControlPose, pickupMiddleSpikeConnectorPose, pickupMiddleSpikeEndPose,
            scoreMiddleSpikeControlPose,
            pickupGateControlPose, pickupGateConnectorPose, pickupGateEndPose, pickupGateIntakePose,
            scoreGateControlPose,
            pickupCloseSpikeConnectorPose, pickupCloseSpikeEndPose,
            pickupFarSpikeControlPose, pickupFarSpikeConnectorPose, pickupFarSpikeEndPose,
            finalPose;

    private PathChain
            scorePreload,
            grabPickupMiddleSpike, scorePickupMiddleSpike,
            grabPickupGate, intakePickupGate, scorePickupGate,
            grabPickupCloseSpike, scorePickupCloseSpike,
            grabPickupFarSpike, scorePickupFarSpike,
            finalPark;

    @Override
    protected void computePoses() {
        // Blue alliance base coordinates mirrored via alliancePose()
        shootPose = alliancePose(new Pose(60, 84, Math.toRadians(135)));

        pickupMiddleSpikeControlPose = alliancePose(new Pose(60, 60));
        pickupMiddleSpikeConnectorPose = alliancePose(new Pose(40, 60, Math.toRadians(0)));
        pickupMiddleSpikeEndPose = alliancePose(new Pose(11, 60, Math.toRadians(0)));
        scoreMiddleSpikeControlPose = alliancePose(new Pose(36, 60));

        pickupGateControlPose = alliancePose(new Pose(60, 63));
        pickupGateConnectorPose = alliancePose(new Pose(40, 63, Math.toRadians(-15)));
        pickupGateEndPose = alliancePose(new Pose(13, 63, Math.toRadians(-15)));
        pickupGateIntakePose = alliancePose(new Pose(10, 58, Math.toRadians(-25)));
        scoreGateControlPose = alliancePose(new Pose(38, 58));

        pickupCloseSpikeConnectorPose = alliancePose(new Pose(40, 84, Math.toRadians(0)));
        pickupCloseSpikeEndPose = alliancePose(new Pose(19, 84, Math.toRadians(0)));

        pickupFarSpikeControlPose = alliancePose(new Pose(60, 37));
        pickupFarSpikeConnectorPose = alliancePose(new Pose(40, 37, Math.toRadians(0)));
        pickupFarSpikeEndPose = alliancePose(new Pose(11, 37, Math.toRadians(0)));

        finalPose = alliancePose(new Pose(60, 110, Math.toRadians(135)));
    }

    @Override
    protected void buildPaths(Pose startPose) {
        scorePreload = robot.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        grabPickupMiddleSpike = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupMiddleSpikeControlPose, pickupMiddleSpikeConnectorPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupMiddleSpikeConnectorPose.getHeading())
                .addPath(new BezierLine(pickupMiddleSpikeConnectorPose, pickupMiddleSpikeEndPose))
                .setConstantHeadingInterpolation(pickupMiddleSpikeEndPose.getHeading())
                .build();

        scorePickupMiddleSpike = robot.pathBuilder()
                .addPath(new BezierCurve(pickupMiddleSpikeEndPose, scoreMiddleSpikeControlPose, shootPose))
                .setLinearHeadingInterpolation(pickupMiddleSpikeEndPose.getHeading(), shootPose.getHeading())
                .build();

        grabPickupGate = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupGateControlPose, pickupGateConnectorPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupGateConnectorPose.getHeading())
                .addPath(new BezierLine(pickupGateConnectorPose, pickupGateEndPose))
                .setConstantHeadingInterpolation(pickupGateEndPose.getHeading())
                .build();

        intakePickupGate = robot.pathBuilder()
                .addPath(new BezierLine(pickupGateEndPose, pickupGateIntakePose))
                .setLinearHeadingInterpolation(pickupGateEndPose.getHeading(), pickupGateIntakePose.getHeading())
                .build();

        scorePickupGate = robot.pathBuilder()
                .addPath(new BezierCurve(pickupGateIntakePose, scoreGateControlPose, shootPose))
                .setLinearHeadingInterpolation(pickupGateIntakePose.getHeading(), shootPose.getHeading())
                .build();

        grabPickupCloseSpike = robot.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupCloseSpikeConnectorPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupCloseSpikeConnectorPose.getHeading())
                .addPath(new BezierLine(pickupCloseSpikeConnectorPose, pickupCloseSpikeEndPose))
                .setConstantHeadingInterpolation(pickupCloseSpikeEndPose.getHeading())
                .build();

        scorePickupCloseSpike = robot.pathBuilder()
                .addPath(new BezierLine(pickupCloseSpikeEndPose, shootPose))
                .setLinearHeadingInterpolation(pickupCloseSpikeEndPose.getHeading(), shootPose.getHeading())
                .build();

        grabPickupFarSpike = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupFarSpikeControlPose, pickupFarSpikeConnectorPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupFarSpikeConnectorPose.getHeading())
                .addPath(new BezierLine(pickupFarSpikeConnectorPose, pickupFarSpikeEndPose))
                .setConstantHeadingInterpolation(pickupFarSpikeEndPose.getHeading())
                .build();

        scorePickupFarSpike = robot.pathBuilder()
                .addPath(new BezierLine(pickupFarSpikeEndPose, shootPose))
                .setLinearHeadingInterpolation(pickupFarSpikeEndPose.getHeading(), shootPose.getHeading())
                .build();

        finalPark = robot.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        setPathState(PathState.INITIAL);
    }

    @Override
    protected void stateMachine() {
        switch ((PathState) pathState) {
            case INITIAL:
                robot.followPath(scorePreload, true);
                robot.spinUp();
                setPathState(PathState.MOVE_PRELOAD);
                break;

            case MOVE_PRELOAD:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.0)) {
                    robot.intake(1);
                    robot.followPath(grabPickupMiddleSpike, true);
                    setPathState(PathState.GRAB_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case GRAB_PICKUP_MIDDLE_SPIKE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case WAIT_PICKUP_MIDDLE_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.75)) {
                    robot.prepareSpinUp();
                    robot.followPath(scorePickupMiddleSpike, true);
                    setPathState(PathState.SCORE_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case SCORE_PICKUP_MIDDLE_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.3)) {
                    robot.spinUp();
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case SHOOT_PICKUP_MIDDLE_SPIKE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.0)) {
                    robot.intake(1);
                    robot.followPath(grabPickupGate, true);
                    setPathState(PathState.GRAB_PICKUP_GATE);
                }
                break;

            case GRAB_PICKUP_GATE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.INTAKE_PICKUP_GATE);
                    robot.followPath(intakePickupGate);
                }
                break;

            case INTAKE_PICKUP_GATE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_PICKUP_GATE);
                }
                break;

            case WAIT_PICKUP_GATE:
                if (Robot.DataPasser.hasElapsed(1.5)) {
                    robot.prepareSpinUp();
                    robot.followPath(scorePickupGate, true);
                    setPathState(PathState.SCORE_PICKUP_GATE);
                }
                break;

            case SCORE_PICKUP_GATE:
                if (Robot.DataPasser.hasElapsed(0.3)) {
                    robot.spinUp();
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PICKUP_GATE);
                }
                break;

            case SHOOT_PICKUP_GATE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.0)) {
                    robot.intake(1);
                    robot.followPath(grabPickupCloseSpike, true);
                    setPathState(PathState.GRAB_PICKUP_CLOSE_SPIKE);
                }
                break;

            case GRAB_PICKUP_CLOSE_SPIKE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_PICKUP_CLOSE_SPIKE);
                }
                break;

            case WAIT_PICKUP_CLOSE_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.75)) {
                    robot.prepareSpinUp();
                    robot.followPath(scorePickupCloseSpike, true);
                    setPathState(PathState.SCORE_PICKUP_CLOSE_SPIKE);
                }
                break;

            case SCORE_PICKUP_CLOSE_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.3)) {
                    robot.spinUp();
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PICKUP_CLOSE_SPIKE);
                }
                break;

            case SHOOT_PICKUP_CLOSE_SPIKE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.0)) {
                    robot.intake(1);
                    robot.followPath(grabPickupFarSpike, true);
                    setPathState(PathState.GRAB_PICKUP_FAR_SPIKE);
                }
                break;

            case GRAB_PICKUP_FAR_SPIKE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_PICKUP_FAR_SPIKE);
                }
                break;

            case WAIT_PICKUP_FAR_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.75)) {
                    robot.prepareSpinUp();
                    robot.followPath(scorePickupFarSpike, true);
                    setPathState(PathState.SCORE_PICKUP_FAR_SPIKE);
                }
                break;

            case SCORE_PICKUP_FAR_SPIKE:
                if (Robot.DataPasser.hasElapsed(0.3)) {
                    robot.spinUp();
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PICKUP_FAR_SPIKE);
                }
                break;

            case SHOOT_PICKUP_FAR_SPIKE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.0)) {
                    robot.followPath(finalPark, true);
                    setPathState(PathState.PARKING);
                }
                break;

            case PARKING:
                robot.stop();
                break;
        }
    }

    protected Pose getStartCalibrationPose() {
        return new Pose(47.17, 133.31, Math.toRadians(180));
    }
}