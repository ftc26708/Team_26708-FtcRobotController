package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Locale;

public class IntakeTransfer {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
    public IntakeTransfer(DcMotorEx intakeMotor, DcMotorEx transferMotor) {
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }
    public void setTransferPower(double power) {
        transferMotor.setPower(power);
    }
    public String getTelemetry() {
        return String.format(Locale.US,
                "Intake: %.2f | Transfer: %.2f",
                intakeMotor.getPower(), transferMotor.getPower()
        );
    }
}