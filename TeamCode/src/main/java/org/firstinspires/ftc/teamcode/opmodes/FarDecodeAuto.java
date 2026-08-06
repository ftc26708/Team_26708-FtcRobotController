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
        PREPARE_PRELOAD,
        SHOOT_PRELOAD,
        INTAKE_SPIKE_MARK,
        PREPARE_SPIKE_MARK,
        WAIT_SPIN_UP_SPIKE_MARK,
        SHOOT_SPIKE_MARK,
        INTAKE_LOADING_ZONE,
        PREPARE_LOADING_ZONE,
        WAIT_SPIN_UP_LOADING_ZONE,
        SHOOT_LOADING_ZONE,
        MOVE_FINAL_PARK,
        IDLE_PARKED
    }

    private Pose
            shootPose,
            spikeMarkControlPose, spikeMarkConnectorPose, spikeMarkEndPose,
            loadingZoneControlPose1, loadingZoneControlPose2,
            loadingZoneConnectorPose, loadingZoneEndPose,
            finalPose;

    private PathChain
            scorePreload,
            pickupSpikeMark, scoreSpikeMark,
            pickupLoadingZone, scoreLoadingZone,
            finalPark;

    long startTime;
    double timeTaken;
    int numberOfLoadingZoneCycles = 0;

    @Override
    protected Pose getStartCalibrationPose() {
        return new Pose(47.17, 8.19, Math.toRadians(-90));
    }

    @Override
    protected void computePoses() {
        // Blue alliance base coordinates mirrored via alliancePose()
        shootPose = alliancePose(new Pose(47.17, 13.00, Math.toRadians(110.16)));

        spikeMarkControlPose = alliancePose(new Pose(55.00, 35.38));
        spikeMarkConnectorPose = alliancePose(new Pose(43.00, 35.38));
        spikeMarkEndPose = alliancePose(new Pose(11.00, 36.00, Math.toRadians(0.00)));

        loadingZoneControlPose1 = alliancePose(new Pose(35.00, 44.00));
        loadingZoneControlPose2 = alliancePose(new Pose(12.00, 38.00));
        loadingZoneConnectorPose = alliancePose(new Pose(12.00, 30.00, Math.toRadians(40.00)));
        loadingZoneEndPose = alliancePose(new Pose(12.00, 12.00, Math.toRadians(0.00)));

        finalPose = alliancePose(new Pose(24.00, 16.00, Math.toRadians(0.00)));
    }

    @Override
    protected void buildPaths(Pose startPose) {
        scorePreload = robot.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pickupSpikeMark = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, spikeMarkControlPose, spikeMarkConnectorPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), spikeMarkConnectorPose.getHeading())
                .addPath(new BezierLine(spikeMarkConnectorPose, spikeMarkEndPose))
                .setConstantHeadingInterpolation(spikeMarkEndPose.getHeading())
                .build();

        scoreSpikeMark = robot.pathBuilder()
                .addPath(new BezierLine(spikeMarkEndPose, shootPose))
                .setLinearHeadingInterpolation(spikeMarkEndPose.getHeading(), shootPose.getHeading())
                .build();

        pickupLoadingZone = robot.pathBuilder()
                .addPath(new BezierCurve(shootPose, loadingZoneControlPose1, loadingZoneControlPose2, loadingZoneConnectorPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), loadingZoneConnectorPose.getHeading())
                .addPath(new BezierLine(loadingZoneConnectorPose, loadingZoneEndPose))
                .setConstantHeadingInterpolation(loadingZoneConnectorPose.getHeading())
                .build();

        scoreLoadingZone = robot.pathBuilder()
                .addPath(new BezierLine(loadingZoneEndPose, shootPose))
                .setLinearHeadingInterpolation(loadingZoneEndPose.getHeading(), shootPose.getHeading())
                .build();

        finalPark = robot.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setConstantHeadingInterpolation(finalPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        startTime = System.nanoTime();
        setPathState(PathState.INITIAL);
    }

    @Override
    protected void stateMachine() {
        switch ((PathState) pathState) {
            case INITIAL:
                robot.spinUp();
                if (Robot.DataPasser.hasElapsed(2)) {
                    robot.followPath(scorePreload, true);
                    setPathState(PathState.PREPARE_PRELOAD);
                }
                break;

            case PREPARE_PRELOAD:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.intake(1, 0.5);
                    robot.followPath(pickupSpikeMark, true);
                    setPathState(PathState.INTAKE_SPIKE_MARK);
                }
                break;

            case INTAKE_SPIKE_MARK:
                if (robot.isNotPathFollowing()) {
                    robot.followPath(scoreSpikeMark, true);
                    robot.prepareSpinUp();
                    setPathState(PathState.PREPARE_SPIKE_MARK);
                }
                break;

            case PREPARE_SPIKE_MARK:
                if (Robot.DataPasser.hasElapsed(0.35)) {
                    robot.spinUp();
                } else {
                    robot.prepareSpinUp();
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_SPIN_UP_SPIKE_MARK);
                }
                break;

            case WAIT_SPIN_UP_SPIKE_MARK:
                robot.spinUp();
                if (Robot.DataPasser.hasElapsed(1.5)) {
                    setPathState(PathState.SHOOT_SPIKE_MARK);
                }
                break;

            case SHOOT_SPIKE_MARK:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    robot.intake(1, 0.5);
                    robot.followPath(pickupLoadingZone, true);
                    setPathState(PathState.INTAKE_LOADING_ZONE);
                }
                break;

            case INTAKE_LOADING_ZONE:
                robot.intake(1, 0.5);
                if (robot.isNotPathFollowing()) {
                    robot.followPath(scoreLoadingZone, true);
                    robot.prepareSpinUp();
                    setPathState(PathState.PREPARE_LOADING_ZONE);
                }
                break;

            case PREPARE_LOADING_ZONE:
                if (Robot.DataPasser.hasElapsed(0.35)) {
                    robot.spinUp();
                } else {
                    robot.prepareSpinUp();
                }
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.WAIT_SPIN_UP_LOADING_ZONE);
                }
                break;

            case WAIT_SPIN_UP_LOADING_ZONE:
                robot.spinUp();
                if (Robot.DataPasser.hasElapsed(1.5)) {
                    setPathState(PathState.SHOOT_LOADING_ZONE);
                }
                break;


            case SHOOT_LOADING_ZONE:
                robot.shoot();
                if (Robot.DataPasser.hasElapsed(0.5)) {
                    numberOfLoadingZoneCycles++;
                    if (numberOfLoadingZoneCycles >= 3) {
                        robot.stop();
                        robot.followPath(finalPark, true);
                        setPathState(PathState.MOVE_FINAL_PARK);
                    } else {
                        robot.intake(1, 0.5);
                        robot.followPath(pickupLoadingZone, true);
                        setPathState(PathState.INTAKE_LOADING_ZONE);
                    }
                }
                break;

            case MOVE_FINAL_PARK:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.IDLE_PARKED);
                    timeTaken = (System.nanoTime() - startTime) / 1000000000.0;
                }
                break;

            case IDLE_PARKED:
                telemetry.addData("Time Finished", timeTaken);
                break;
        }
    }
}