package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
public abstract class BaseDecodeAuto extends OpMode {
    protected Robot robot;
    protected Enum<?> pathState;
    public enum InitState { HARDWARE, SELECT_ALLIANCE, COMPUTE_POSES, BUILD_PATHS, READY }
    private InitState initState = InitState.HARDWARE;

    public void init() {}

    @Override
    public void init_loop() {
        switch (initState) {
            case HARDWARE:
                robot = new Robot(hardwareMap);
                initState = InitState.SELECT_ALLIANCE;
                break;
            case SELECT_ALLIANCE:
                telemetry.addLine("LB: BLUE | RB: RED");
                if (gamepad1.left_bumper) robot.setAlliance(Robot.DataPasser.Alliance.BLUE);
                else if (gamepad1.right_bumper) robot.setAlliance(Robot.DataPasser.Alliance.RED);

                if (Robot.DataPasser.currentAlliance != Robot.DataPasser.Alliance.UNKNOWN)
                    initState = InitState.COMPUTE_POSES;
                break;
            case COMPUTE_POSES:
                computePoses();
                initState = InitState.BUILD_PATHS;
                break;
            case BUILD_PATHS:
                buildPaths();
                initState = InitState.READY;
                break;
            case READY:
                telemetry.addLine("READY - Press Play");
                break;
        }
        telemetry.update();
    }

    @Override
    public void loop() {
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
    protected abstract void buildPaths();
    protected abstract void stateMachine();
}