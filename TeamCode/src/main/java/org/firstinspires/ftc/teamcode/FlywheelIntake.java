package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelIntake {
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    public FlywheelIntake(HardwareMap hardwareMap){
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
    }

    public void flywheelOn() {
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
    }

    public void flywheelOff() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void flywheelSpitOut(){
        leftIntake.setPower(-1);
        rightIntake.setPower(1);
    }

}
