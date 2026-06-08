package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Locale;

public class IntakeTransfer {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
    public IntakeTransfer(DcMotorEx intakeMotor, DcMotorEx transferMotor) {
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients coefficients = new PIDFCoefficients(30, 0, 0, 15);
        intakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        transferMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveIntake(double power) {
        if (intakeMotor.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        intakeMotor.setPower(power);
    }
    public void moveTransfer(double power) {
        if (transferMotor.getMode() != DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        transferMotor.setPower(power);
    }
    public void positionIntake(double turns) {
        if (intakeMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        int ticks = (int) (turns * ((1 + (46.0 / 11.0)) * 28));
        intakeMotor.setTargetPosition(ticks);
        intakeMotor.setPower(1);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void positionTransfer(double turns) {
        if (transferMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            transferMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        int ticks = (int) (turns * ((1 + (46.0 / 11.0)) * 28));
        transferMotor.setTargetPosition(ticks);
        transferMotor.setPower(1);
        transferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public String getTelemetry() {
        return String.format(Locale.US,
                "Intake: %.2f | Transfer: %.2f",
                intakeMotor.getPower(), transferMotor.getPower()
        );
    }
}