package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "DECODE Autonomous (Far)", preselectTeleOp = "DECODE TeleOp")
public class FarDecodeAuto extends BaseDecodeAuto {

    public enum PathState {
        INITIAL,
        MOVE_PRELOAD,
        SHOOT_PRELOAD,
        GRAB_SPIKE_MARK,
        WAIT_SPIKE_MARK,
        MOVE_SPIKE_MARK,
        SHOOT_SPIKE_MARK,
        MOVE_OPTIONAL_PARK,
        WAIT_OPTIONAL_PARK,
        START_GRAB_LOADING_ZONE,
        END_GRAB_LOADING_ZONE,
        WAIT_LOADING_ZONE,
        MOVE_LOADING_ZONE,
        SHOOT_LOADING_ZONE,
        MOVE_FINAL_PARK,
        IDLE_PARKED
    }

    private Pose
            shootPose, optionalParkPose,
            spikeMarkControlPose1, spikeMarkControlPose2, spikeMarkControlPose3, spikeMarkEndPose,
            loadingZoneStartPose, loadingZoneEndPose,
            finalPose;

    private PathChain
            scorePreload, startLoadingZonePickup, endLoadingZonePickup, scoreLoadingZonePickup,
            scoreOptionalPark, grabSpikeMarkPickup, scoreSpikeMarkPickup,
            scoreFinalPark;

    @Override
    protected void computePoses() {
        // Blue alliance base coordinates mirrored via alliancePose()
        shootPose = alliancePose(new Pose(60.000, 21.000, Math.toRadians(116)));

        loadingZoneStartPose = alliancePose(new Pose(11.000, 21.000, Math.toRadians(20)));
        loadingZoneEndPose = alliancePose(new Pose(11.000, 11.000, Math.toRadians(20)));

        optionalParkPose = alliancePose(new Pose(36.000, 9.000, Math.toRadians(0)));

        spikeMarkControlPose1 = alliancePose(new Pose(60.000, 24.000));
        spikeMarkControlPose2 = alliancePose(new Pose(60.000, 36.000));
        spikeMarkControlPose3 = alliancePose(new Pose(36.000, 36.000));
        spikeMarkEndPose = alliancePose(new Pose(9.000, 36.000, Math.toRadians(0)));

        finalPose = alliancePose(new Pose(54.000, 33.300, Math.toRadians(116)));
    }

    @Override
    protected void buildPaths(Pose startPose) {
        scorePreload = robot.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        startLoadingZonePickup = robot.pathBuilder()
                .addPath(new BezierLine(shootPose, loadingZoneStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), loadingZoneStartPose.getHeading())
                .build();

        endLoadingZonePickup = robot.pathBuilder()
                .addPath(new BezierLine(loadingZoneStartPose, loadingZoneEndPose))
                .setConstantHeadingInterpolation(loadingZoneEndPose.getHeading())
                .build();

        scoreLoadingZonePickup = robot.pathBuilder()
                .addPath(new BezierLine(loadingZoneEndPose, shootPose))
                .setLinearHeadingInterpolation(loadingZoneEndPose.getHeading(), shootPose.getHeading())
                .build();

        scoreOptionalPark = robot.pathBuilder()
                .addPath(new BezierLine(shootPose, optionalParkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), optionalParkPose.getHeading())
                .build();

        grabSpikeMarkPickup = robot.pathBuilder()
                .addPath(new BezierCurve(optionalParkPose, spikeMarkControlPose1, spikeMarkControlPose2, spikeMarkControlPose3, spikeMarkEndPose))
                .setConstantHeadingInterpolation(spikeMarkEndPose.getHeading())
                .build();

        scoreSpikeMarkPickup = robot.pathBuilder()
                .addPath(new BezierLine(spikeMarkEndPose, shootPose))
                .setLinearHeadingInterpolation(spikeMarkEndPose.getHeading(), shootPose.getHeading())
                .build();

        scoreFinalPark = robot.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setConstantHeadingInterpolation(finalPose.getHeading())
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
                if (Robot.DataPasser.hasElapsed(1.5)) {
                    robot.intake(1);
                    robot.followPath(startLoadingZonePickup, 0.6, true);
                    setPathState(PathState.START_GRAB_LOADING_ZONE);
                }
                break;

            case START_GRAB_LOADING_ZONE:
                if (robot.isNotPathFollowing()) {
                    robot.followPath(endLoadingZonePickup, 0.4, true);
                    setPathState(PathState.END_GRAB_LOADING_ZONE);
                }
                break;

            case END_GRAB_LOADING_ZONE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_LOADING_ZONE);
                }
                break;

            case WAIT_LOADING_ZONE:
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.followPath(scoreLoadingZonePickup, true);
                    setPathState(PathState.MOVE_LOADING_ZONE);
                }
                break;

            case MOVE_LOADING_ZONE:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_LOADING_ZONE);
                }
                break;

            case SHOOT_LOADING_ZONE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.5)) {
                    robot.intake(1);
                    robot.followPath(scoreOptionalPark, true);
                    setPathState(PathState.MOVE_OPTIONAL_PARK);
                }
                break;

            case MOVE_OPTIONAL_PARK:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_OPTIONAL_PARK);
                }
                break;

            case WAIT_OPTIONAL_PARK:
                if (Robot.DataPasser.hasElapsed(3.0)) {
                    robot.followPath(grabSpikeMarkPickup, 0.6, true);
                    setPathState(PathState.GRAB_SPIKE_MARK);
                }
                break;

            case GRAB_SPIKE_MARK:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_SPIKE_MARK);
                }
                break;

            case WAIT_SPIKE_MARK:
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.followPath(scoreSpikeMarkPickup, true);
                    setPathState(PathState.MOVE_SPIKE_MARK);
                }
                break;

            case MOVE_SPIKE_MARK:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_SPIKE_MARK);
                }
                break;

            case SHOOT_SPIKE_MARK:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(1.5)) {
                    robot.stop(); // Stops intake/shooters/transfer
                    robot.followPath(scoreFinalPark, true);
                    setPathState(PathState.MOVE_FINAL_PARK);
                }
                break;

            case MOVE_FINAL_PARK:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.IDLE_PARKED);
                }
                break;

            case IDLE_PARKED:
                break;
        }
    }

    protected Pose getStartCalibrationPose() {
        return new Pose(60.000, 7.950, Math.toRadians(180));
    }
}