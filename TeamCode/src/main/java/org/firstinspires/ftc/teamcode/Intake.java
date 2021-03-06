package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor intakeMotor; //intake

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void intakeOn() {
        intakeMotor.setPower(-1);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Surgical Tubing Motor Power", intakeMotor.getPower());
    }
}
