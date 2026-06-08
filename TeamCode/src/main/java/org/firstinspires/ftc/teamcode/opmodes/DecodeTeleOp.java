package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "DECODE TeleOp")
public class DecodeTeleOp extends OpMode {
    private Robot robot;

    private boolean relocalizedThisCycle = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;
    private boolean pathAlreadyFollowed = false;
    boolean shooting = false;

    private enum DriveMode { MANUAL, SHOOT, PARK }
    private DriveMode driveMode = DriveMode.MANUAL;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.startTeleopDrive();

        // Initialize from Auto
        robot.setAlliance(Robot.DataPasser.currentAlliance);
        robot.setStartingPose(Robot.DataPasser.endAutoPose);
    }

    public void loop() {
        robot.clearCache();

        robot.updateDrivetrain();

        localizationLogic();
        drivetrainLogic();
        mechanismLogic();

        robot.telemetryOutput(telemetry);
        telemetry.update();
    }

    private void localizationLogic() {
        if (gamepad1.dpad_left) {
            robot.setAlliance(Robot.DataPasser.Alliance.BLUE);
        }
        if (gamepad1.dpad_right) {
            robot.setAlliance(Robot.DataPasser.Alliance.RED);
        }

        Pose currentPose = robot.getPose();

        if (gamepad2.dpad_left && !lastDpadLeft) {
            robot.setPose(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() + Math.toRadians(0.5)));
        }
        if (gamepad2.dpad_right && !lastDpadRight) {
            robot.setPose(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading() - Math.toRadians(0.5)));
        }
        if (gamepad2.dpad_up) {
            robot.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(90)));
        }
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;

        if (gamepad2.right_trigger > 0.1 && !relocalizedThisCycle) {
            if (robot.relocalizeOdoWithCamera()) {
                relocalizedThisCycle = true;
            }
        }

        if (gamepad2.right_trigger < 0.1) {
            relocalizedThisCycle = false;
        }
    }

    private void drivetrainLogic() {
        DriveMode prevMode = driveMode;
        if (gamepad1.a) driveMode = DriveMode.PARK;
        if (gamepad1.y) driveMode = DriveMode.SHOOT;

        // Manual override
        if ((Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) && driveMode != DriveMode.MANUAL) {
            driveMode = DriveMode.MANUAL;
        }

        if (prevMode != driveMode) {
            robot.breakPathFollowing();
            if (driveMode == DriveMode.MANUAL) {
                robot.startTeleopDrive();
            }
            pathAlreadyFollowed = false;
        }

        switch (driveMode) {
            case MANUAL:
                double translationSpeedMult = 0.30;
                double rotationSpeedMult = 0.30;

                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    translationSpeedMult = 0.15;
                    rotationSpeedMult = 0.15;
                }
                if (gamepad1.left_stick_button) { translationSpeedMult = 1.0; }
                if (gamepad1.right_stick_button) { rotationSpeedMult = 1.0; }

                double forward = gamepad1.left_stick_y * translationSpeedMult;
                double strafe = gamepad1.left_stick_x * translationSpeedMult;
                double turn = -gamepad1.right_stick_x * rotationSpeedMult;

                if (Robot.DataPasser.currentAlliance == Robot.DataPasser.Alliance.RED) {
                    forward = -forward;
                    strafe = -strafe;
                }

                if (gamepad2.right_trigger > 0.1) {
                    robot.autoAimDrive(forward, strafe);
                } else {
                    robot.manualDrive(forward, strafe, turn);
                }
                break;

            case PARK:
                if (robot.isNotPathFollowing() && !pathAlreadyFollowed) {
                    Pose parkPose = (Robot.DataPasser.currentAlliance == Robot.DataPasser.Alliance.RED)
                            ? new Pose(38, 32.75, Math.toRadians(90))
                            : new Pose(103.5, 32.75, Math.toRadians(90));

                    followAutomatedPath(parkPose);
                    pathAlreadyFollowed = true;
                }
                break;

            case SHOOT:
                if (robot.isNotPathFollowing() && !pathAlreadyFollowed) {
                    Pose shootPose = (Robot.DataPasser.currentAlliance == Robot.DataPasser.Alliance.RED)
                            ? new Pose(94.333, 94.333, Math.toRadians(135))
                            : new Pose(47.167, 94.333, Math.toRadians(45));

                    followAutomatedPath(shootPose);
                    pathAlreadyFollowed = true;
                }
                break;
        }
    }

    private void followAutomatedPath(Pose target) {
        PathChain path = robot.pathBuilder()
                .addPath(new BezierLine(robot.getPose(), target))
                .setGlobalLinearHeadingInterpolation(robot.getPose().getHeading(), target.getHeading(), 0.5)
                .build();
        robot.followPath(path, true);
    }

    private void mechanismLogic() {
        if (gamepad2.right_trigger > 0.1 || gamepad2.right_bumper) {
            if (robot.isReadyToShoot() || shooting || gamepad2.right_bumper) {
                robot.shoot();
                shooting = true;
            } else {
                robot.spinUp();
            }
        } else {
            double intakePower = -gamepad2.left_stick_y;
            double transferPower = gamepad2.left_bumper ? -1.0 : (1.2 * gamepad2.left_trigger - 0.2);

            double shooterRPM = 0;
            if (gamepad2.a) shooterRPM = -1800;
            else if (gamepad2.b) shooterRPM = 1200;
            else if (gamepad2.x) shooterRPM = 1350;
            else if (gamepad2.y) shooterRPM = 1700;

            robot.customMechSpeeds(intakePower, transferPower, shooterRPM);

            shooting = false;
        }
    }
}