package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SampleDrive {
    DcMotorEx FRMotor;
    DcMotorEx FLMotor;
    DcMotorEx BRMotor;
    DcMotorEx BLMotor;

    private DistanceSensor sensorDistanceL; //left front sensor
    ModernRoboticsI2cRangeSensor sensorRangeM;
    private DistanceSensor sensorDistanceR; //right front sensor

    Carousel carousel;
    Catapult catapult;
    Intake intake;
    Lift lift;

    public SampleDrive(HardwareMap hardwareMap) {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");

        carousel = new Carousel(hardwareMap);
        catapult = new Catapult(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
    }

    public void drive(double x, double y, double rotation) {
        FRMotor.setPower(-x - y - rotation);
        FLMotor.setPower(-x + y - rotation);
        BRMotor.setPower(-x + y + rotation);
        BLMotor.setPower(-x - y + rotation);

        //FRMotor.setVelocity((x - y + rotation) * 1000);
        //FLMotor.setVelocity((x + y + rotation) * 1000);
        //BRMotor.setVelocity((-x - y + rotation) * 1000);
        //BLMotor.setVelocity((-x + y + rotation) * 1000);
    }

    public void setPos(double x, double y, double rotation, Telemetry telemetry) {
        FRMotor.setTargetPosition((int)(-x + y - rotation) + FRMotor.getCurrentPosition());
        FLMotor.setTargetPosition((int)(-x - y - rotation) + FLMotor.getCurrentPosition());
        BRMotor.setTargetPosition((int)(-x - y + rotation) + BRMotor.getCurrentPosition());
        BLMotor.setTargetPosition((int)(-x + y + rotation) + BLMotor.getCurrentPosition());
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setPower(0.5);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setPower(0.5);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setPower(0.5);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setPower(0.5);
        while(FRMotor.isBusy() && FLMotor.isBusy() && BRMotor.isBusy() && BLMotor.isBusy()) { telemetry(telemetry); }
    }

    public void moveDistance(double power, int distance) { //idk in progres
        FLMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRMotor.setTargetPosition(distance);
        FLMotor.setTargetPosition(distance);
        BRMotor.setTargetPosition(distance);
        BLMotor.setTargetPosition(distance);

        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(FRMotor.isBusy() && FLMotor.isBusy() && BRMotor.isBusy() && BLMotor.isBusy()) { }


        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward(double power) {
        FRMotor.setPower(power);
        FLMotor.setPower(power);
        BRMotor.setPower(power);
        BLMotor.setPower(power);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
    }
}
