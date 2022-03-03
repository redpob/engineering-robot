package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Catapult {
    private DcMotorEx catapultMotor; //outtake
    private Servo headServo; //catapult flap
    private Servo flapServo; //catapult flap
    private int startPosition;

    public Catapult(HardwareMap hardwareMap) {
        catapultMotor = hardwareMap.get(DcMotorEx.class, "catapult");
        headServo = hardwareMap.get(Servo.class, "head");
        flapServo = hardwareMap.get(Servo.class, "flap");
        startPosition = catapultMotor.getCurrentPosition();
        catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        catapultMotor.setPower(power);
    }

    public void upper() {//12.2 in
        catapultMotor.setTargetPosition(140 + startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(catapultMotor.getCurrentPosition() > startPosition + 80 && catapultMotor.getCurrentPosition() < catapultMotor.getTargetPosition() - 10) {
            catapultMotor.setPower(0);
        } else {
            catapultMotor.setPower(1);
        }
    }

    public void middle() {//13 in
        catapultMotor.setTargetPosition(155 + startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(catapultMotor.getCurrentPosition() > startPosition + 80 && catapultMotor.getCurrentPosition() < catapultMotor.getTargetPosition() - 10) {
            catapultMotor.setPower(0);
        } else {
            catapultMotor.setPower(1);
        }
    }

    public void lower() {//14 in
        catapultMotor.setTargetPosition(175 + startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(catapultMotor.getCurrentPosition() > startPosition + 80 && catapultMotor.getCurrentPosition() < catapultMotor.getTargetPosition() - 10) {
            catapultMotor.setPower(0);
        } else {
            catapultMotor.setPower(1);
        }
    }

    public void returnPosition() {
        catapultMotor.setTargetPosition(startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(catapultMotor.getCurrentPosition() < startPosition + 80) {
            catapultMotor.setPower(0);
        } else {
            catapultMotor.setPower(1);
        }
    }

    public double getVelocity() {
        return catapultMotor.getVelocity();
    }

    public void headFold() {
        headServo.setPosition(0);
    }

    public void headUnfold() {
        headServo.setPosition(1);
    }

    public void flapOpen() {
        flapServo.setPosition(0);
    }

    public void flapClose() {
        flapServo.setPosition(0.5);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Catapult Motor Current", catapultMotor.getCurrentPosition());
        telemetry.addData("Catapult Motor Target", catapultMotor.getTargetPosition());
        telemetry.addData("Catapult Head Position", headServo.getPosition());
        telemetry.addData("Catapult Flap Position", flapServo.getPosition());
    }
}
