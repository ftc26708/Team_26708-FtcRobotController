package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
public abstract class BaseDecodeAuto extends OpMode {
    protected Robot robot;
    protected Enum<?> pathState;
    private Pose calibratedStartPose;
    public enum InitState { HARDWARE, SELECT_ALLIANCE, CALIBRATE_POSITION, COMPUTE_POSES, BUILD_PATHS, READY }
    private InitState initState = InitState.HARDWARE;

    public void init() {}

    @Override
    public void init_loop() {
        switch (initState) {
            case HARDWARE:
                robot = new Robot(hardwareMap);
                Robot.DataPasser.currentAlliance = Robot.DataPasser.Alliance.UNKNOWN;
                initState = InitState.SELECT_ALLIANCE;
                break;
            case SELECT_ALLIANCE:
                robot.clearCache();
                telemetry.addLine("POSITION IN CALIBRATION POSITION OF THIS AUTO. \nTHEN, PRESS LB FOR BLUE OR RB FOR RED.");
                if (gamepad1.left_bumper) robot.setAlliance(Robot.DataPasser.Alliance.BLUE);
                else if (gamepad1.right_bumper) robot.setAlliance(Robot.DataPasser.Alliance.RED);
                if (Robot.DataPasser.currentAlliance != Robot.DataPasser.Alliance.UNKNOWN)
                    initState = InitState.CALIBRATE_POSITION;
                break;
            case CALIBRATE_POSITION:
                robot.clearCache();
                telemetry.addLine("PRESS A WHEN POSITIONED IN THE INTENDED START POSITION.");
                robot.setPose(alliancePose(getStartCalibrationPose()));
                robot.updateDrivetrain();
                if(gamepad1.a) {
                    calibratedStartPose = robot.getPose();
                    initState = InitState.COMPUTE_POSES;
                }
            case COMPUTE_POSES:
                robot.clearCache();
                computePoses();
                initState = InitState.BUILD_PATHS;
                break;
            case BUILD_PATHS:
                robot.clearCache();
                buildPaths(calibratedStartPose);
                initState = InitState.READY;
                break;
            case READY:
                robot.clearCache();
                telemetry.addLine("READY - Press Play");
                break;
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.clearCache();

        robot.updateDrivetrain();

        stateMachine();

        robot.telemetryOutput(telemetry);
        telemetry.addData("Current State", pathState);
        telemetry.update();
    }

    protected void setPathState(Enum<?> newState) {
        pathState = newState;
        Robot.DataPasser.resetTimer();
    }

    protected Pose alliancePose(Pose bluePose) {
        return (Robot.DataPasser.currentAlliance == Robot.DataPasser.Alliance.RED)
                ? bluePose.mirror() : bluePose;
    }

    @Override
    public void stop() {
        Robot.DataPasser.endAutoPose = robot.getPose();
    }

    protected abstract void computePoses();
    protected abstract void buildPaths(Pose calibratedStartPose);
    protected abstract void stateMachine();
    protected abstract Pose getStartCalibrationPose();
}