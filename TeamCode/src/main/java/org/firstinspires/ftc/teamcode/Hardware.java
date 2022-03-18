package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hardware {
    //Motors
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    //Servos
    public Servo randomServo;

    //Additional
    HardwareMap hardwareMap;
    public ElapsedTime runtime = new ElapsedTime();

    public Hardware(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //Connecting Motor
        FRMotor = hwMap.get(DcMotorEx.class, "FR");
        FLMotor = hwMap.get(DcMotorEx.class, "FL");
        BRMotor = hwMap.get(DcMotorEx.class, "BR");
        BLMotor = hwMap.get(DcMotorEx.class, "BL");

        //Connecting servo
        randomServo = hardwareMap.get(Servo.class, "servo");

        //Set Motor Direction
        FRMotor.setDirection(DcMotorEx.Direction.FORWARD);
        FLMotor.setDirection(DcMotorEx.Direction.REVERSE);
        BRMotor.setDirection(DcMotorEx.Direction.FORWARD);
        BLMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //Set Motor Mode
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Zero Power Behavior
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set motors to no power on initialization
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
    }

    public void drive(double x, double y) {
        FRMotor.setPower(-x - y);
        FLMotor.setPower(-x + y);
        BRMotor.setPower(-x - y);
        BLMotor.setPower(-x + y);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
    }
}
