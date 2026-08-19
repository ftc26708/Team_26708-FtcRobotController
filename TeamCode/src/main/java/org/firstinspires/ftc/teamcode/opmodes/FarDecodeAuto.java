package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "DECODE Autonomous (Far)", preselectTeleOp = "DECODE TeleOp")
public class FarDecodeAuto extends BaseDecodeAuto {
    public enum PathState {
        INITIAL,
        PARKING,
        IDLE
    }

    private Pose controlPose, parkPose;

    private PathChain park;

    long startTime;
    double timeTaken;

    @Override
    protected Pose getStartCalibrationPose() {
        return new Pose(47.17, 8.19, Math.toRadians(-90));
    }

    @Override
    protected void computePoses() {
        controlPose = alliancePose(new Pose(42, 15));
        parkPose = alliancePose(new Pose(18, 9, Math.toRadians(0)));

    }

    @Override
    protected void buildPaths(Pose startPose) {
        park = robot.pathBuilder()
                .addPath(new BezierCurve(startPose, controlPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading(), 0.75)
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
                robot.followPath(park, true);
                setPathState(PathState.PARKING);

            case PARKING:
                if (robot.isNotPathFollowing()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                telemetry.addData("Time Finished", timeTaken);
                break;
        }
    }
}