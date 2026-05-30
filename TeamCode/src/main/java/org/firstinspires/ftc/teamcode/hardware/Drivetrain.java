package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import java.util.Locale;

public class Drivetrain {
    private final Follower follower;
    private boolean fieldCentric = true;
    private double x, y, turn;
    public Drivetrain(Follower follower) {
        this.follower = follower;
    }

    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }
    public void update() {
        follower.update();
    }
    public void drive(double x, double y, double turn) {
        follower.setTeleOpDrive(x, y, turn, !fieldCentric);
        this.x = x;
        this.y = y;
        this.turn = turn;
    }
    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }
    public void followPath(Path path) {
        follower.followPath(path);
    }
    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }
    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        follower.followPath(pathChain, maxPower, holdEnd);
    }
    public boolean isBusy() {
        return follower.isBusy();
    }
    public void breakFollowing() {
        follower.breakFollowing();
    }
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }
    public boolean getFieldCentric() {
        return fieldCentric;
    }
    public String getTelemetry() {
        String pathName = isBusy() ? follower.getCurrentPath().toString() : "None";
        return String.format(
                Locale.US,
                "Mode: %s | FieldCentric: %b\nDrive: [%.2f, %.2f] | Turn: %.2f",
                pathName, getFieldCentric(), x, y, turn
        );
    }
}