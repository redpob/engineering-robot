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

public class SampleDriveBigBird {
    DcMotorEx FRMotor;
    DcMotorEx FLMotor;
    DcMotorEx BRMotor;
    DcMotorEx BLMotor;

    private DistanceSensor sensorDistanceL; //left front sensor
    ModernRoboticsI2cRangeSensor sensorRangeM;
    private DistanceSensor sensorDistanceR; //right front sensor

    public SampleDriveBigBird(HardwareMap hardwareMap) {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
    }

    public void drive(double x, double y, double rotation) {
        y = -y;
        FRMotor.setPower(x + y + rotation);
        FLMotor.setPower(x - y + rotation);
        BRMotor.setPower(-x + y + rotation);
        BLMotor.setPower(-x - y + rotation);

        //FRMotor.setVelocity((x - y + rotation) * 1000);
        //FLMotor.setVelocity((x + y + rotation) * 1000);
        //BRMotor.setVelocity((-x - y + rotation) * 1000);
        //BLMotor.setVelocity((-x + y + rotation) * 1000);
    }

    public void setPos(double x, double y, double rotation) {
        FRMotor.setTargetPosition((int)( x - y + rotation) + FRMotor.getCurrentPosition());
        FLMotor.setTargetPosition((int)( x + y + rotation) + FLMotor.getCurrentPosition());
        BRMotor.setTargetPosition((int)(-x - y + rotation) + BRMotor.getCurrentPosition());
        BLMotor.setTargetPosition((int)(-x + y + rotation) + BLMotor.getCurrentPosition());
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setPower(0.2);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setPower(0.2);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setPower(0.2);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setPower(0.2);
        while(!atTarget()) { }
    }

    public boolean atTarget() {
        if(FRMotor.getCurrentPosition() > FRMotor.getTargetPosition() - 10 && FRMotor.getCurrentPosition() < FRMotor.getTargetPosition() + 10 &&
                FLMotor.getCurrentPosition() > FLMotor.getTargetPosition() - 10 && FLMotor.getCurrentPosition() < FLMotor.getTargetPosition() + 10 &&
                BRMotor.getCurrentPosition() > BRMotor.getTargetPosition() - 10 && BRMotor.getCurrentPosition() < BRMotor.getTargetPosition() + 10 &&
                BLMotor.getCurrentPosition() > BLMotor.getTargetPosition() - 10 && BLMotor.getCurrentPosition() < BLMotor.getTargetPosition() + 10)
            return true;
        else
            return false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left 2MDistance Sensor Range", String.format("%.01f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Middle Range Sensor Range", String.format("%.01f in", sensorRangeM.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Right 2MDistance Sensor Range", String.format("%.01f in", sensorDistanceR.getDistance(DistanceUnit.INCH)));
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
        telemetry.update();
    }
}
