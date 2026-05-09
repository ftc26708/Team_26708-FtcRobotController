package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Locale;

public class Shooter {
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final double TPS_TO_RPM = 15.0 / 7.0;
    private double targetVelocity;

    public Shooter(DcMotorEx leftShooter, DcMotorEx rightShooter) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;

        PIDFCoefficients coefficients = new PIDFCoefficients(90.0, 0.02, 2.5, 13.2);
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx motor : shooters) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        }

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setTargetVelocity(double velocity) {
        leftShooter.setVelocity(velocity / TPS_TO_RPM);
        rightShooter.setVelocity(velocity / TPS_TO_RPM);
        targetVelocity = velocity;
    }

    public double getActualVelocity() {
        return TPS_TO_RPM * (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
    }

    public double getVelocityError() {
        return targetVelocity - getActualVelocity();
    }

    public double getNeededVelocity(double distance) {
        double A = 0.033;
        double B = 0;
        double C = 1045;
        return ((A * Math.pow(distance, 2)) + (B * distance) + C);
    }

    public String getTelemetry() {
        return String.format(Locale.US,
                "T: %.0f | A: %.0f (L: %.0f, R: %.0f) | E: %.0f",
                targetVelocity, getActualVelocity(),
                leftShooter.getVelocity() * TPS_TO_RPM,
                rightShooter.getVelocity() * TPS_TO_RPM,
                getVelocityError()
        );
    }
}