package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private DcMotorEx liftMotor;
    private int motorStartPosition;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        motorStartPosition = liftMotor.getCurrentPosition();
    }

    public void lift() {
        liftMotor.setTargetPosition(1000 + motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void lower() {
        liftMotor.setTargetPosition(motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lift Motor Current", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Motor Target", liftMotor.getTargetPosition());
    }
}
