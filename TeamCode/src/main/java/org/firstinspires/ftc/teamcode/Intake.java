package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor surgicalTubingMotor; //intake
    private Servo intakeHolderServo; //intake holder
    //private TouchSensor intakeTouch; //touch sensor inside of intake

    public Intake(HardwareMap hardwareMap){
        surgicalTubingMotor = hardwareMap.get(DcMotor.class, "surgical_tubing");
        intakeHolderServo = hardwareMap.get(Servo.class, "intake_holder");
        //intakeTouch = hardwareMap.get(TouchSensor.class, "intake_touch");
    }

    public void surgicalTubingOn() {
        surgicalTubingMotor.setPower(-250/300);
    }

    public void surgicalTubingOff() {
        surgicalTubingMotor.setPower(0);
    }

    public void returnIntakeHolder() {
        intakeHolderServo.setPosition(1);
    }

    public void dropIntake() {
        intakeHolderServo.setPosition(0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Surgical Tubing Motor Power", surgicalTubingMotor.getPower());
        telemetry.addData("Intake Holder Servo Position", intakeHolderServo.getPosition());
    }

    public void testMotor(double power) {
        surgicalTubingMotor.setPower(power);
    }
}
