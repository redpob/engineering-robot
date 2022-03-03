package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class MainWestCoast extends OpMode {
    WestCoastDrive drive;
    Catapult catapult;
    Carousel carousel;
    FlywheelIntake intake;
    Lift lift;

    private int intakeCounter= 0;
    private boolean intaking= false;

    private DistanceSensor sensorDistanceL; //left front sensor
    private ModernRoboticsI2cRangeSensor sensorRangeM; //middle front range sensor
    private DistanceSensor sensorDistanceR; //right front sensor

    private boolean lastPressedCatapultUpper = false;
    private boolean lastPressedCatapultMiddle = false;
    private boolean lastPressedCatapultLower = false;
    private boolean catapultUpperToggle = false;
    private boolean catapultMiddleToggle = false;
    private boolean catapultLowerToggle = false;
    private boolean lastPressedFlap = false;
    private boolean flapToggle = false;
    private boolean lastPressedCatapultHeadLeft = false;
    private boolean catapultHeadLeftToggle = false;
    private boolean lastPressedCatapultHeadRight = false;
    private boolean catapultHeadRightToggle = false;

    private boolean lastPressedFlywheel = false;
    private boolean flywheelToggle = false;
    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;
    private boolean lastPressedLiftServo = false;
    private boolean liftServoToggle = false;

    @Override
    public void init(){
        drive = new WestCoastDrive(hardwareMap);
        catapult = new Catapult(hardwareMap);
        carousel = new Carousel(hardwareMap);
        intake = new FlywheelIntake(hardwareMap);
        lift = new Lift(hardwareMap);

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");

    }

    @Override
    public void loop(){
        drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        drive.telemetry(telemetry);

        //catapult
        if (gamepad1.y && !lastPressedCatapultUpper) {
            catapultUpperToggle = !catapultUpperToggle;
            catapultMiddleToggle = false;
            catapultLowerToggle = false;
        } else if (gamepad1.b && !lastPressedCatapultMiddle) {
            catapultMiddleToggle = !catapultMiddleToggle;
            catapultUpperToggle = false;
            catapultLowerToggle = false;
        } else if (gamepad1.a && !lastPressedCatapultLower) {
            catapultLowerToggle = !catapultLowerToggle;
            catapultUpperToggle = false;
            catapultMiddleToggle = false;
        }
        if (catapultUpperToggle) {
            catapult.upper();
        } else if (catapultMiddleToggle) {
            catapult.middle();
        } else if (catapultLowerToggle) {
            catapult.lower();
        } else {
            catapult.returnPosition();
        }
        lastPressedCatapultUpper = gamepad1.y;
        lastPressedCatapultMiddle = gamepad1.b;
        lastPressedCatapultLower = gamepad1.a;

        //turn flap
        if(gamepad1.x && !lastPressedFlap) {
            flapToggle = !flapToggle;
        }
        if(flapToggle) {
            catapult.flapOpen();
        }
        else {
            catapult.flapClose();
        }
        lastPressedFlap = gamepad1.x;

        //catapult head
        if (gamepad2.dpad_left && !lastPressedCatapultHeadLeft) {
            catapultHeadLeftToggle = !catapultHeadLeftToggle;
            catapultHeadRightToggle = false;
        } else if (gamepad2.dpad_right && !lastPressedCatapultHeadRight) {
            catapultHeadRightToggle = !catapultHeadRightToggle;
            catapultHeadLeftToggle = false;
        }
        if (catapultHeadLeftToggle) {
            catapult.headFold();
        } else {
            catapult.headUnfold();
        }
        catapultHeadLeftToggle = gamepad2.dpad_left;
        catapultHeadRightToggle = gamepad2.dpad_right;

        //team shipping element lift
        if (gamepad1.left_bumper && !lastPressedLiftMotor) {
            liftMotorToggle = !liftMotorToggle;
        }
        if (liftMotorToggle) {
            lift.lift();
        } else {
            lift.lower();
        }
        lastPressedLiftMotor = gamepad1.left_bumper;

        if (gamepad1.left_trigger > 0 && !lastPressedLiftServo) {
            liftServoToggle = !liftServoToggle;
        }
        if (liftMotorToggle) {
            lift.open();
        } else {
            lift.close();
        }
        lastPressedLiftServo = (gamepad1.left_trigger > 0);

        //flywheel
        if (gamepad1.right_bumper && !lastPressedFlywheel) {
            flywheelToggle = !flywheelToggle;
        }
        if (flywheelToggle) {
            intake.flywheelOn();
        } else {
            intake.flywheelOff();
        }
        lastPressedFlywheel = gamepad1.right_bumper;

        //spinning carousel
        if(gamepad1.dpad_left) {
            carousel.spinRed();
        } else if(gamepad1.dpad_right) {
            carousel.spinBlue();
        } else {
            carousel.spinOff();
        }
        //to fix from here
        /*telemetry.update();

        if(gamepad1.x && intakeCounter== 0){
            intakeCounter= 300;
            intaking= true;
        }
        if(intakeCounter>0){
            intakeCounter=intakeCounter-1;
        }
        telemetry.addData("Intake Status", intakeCounter);
        telemetry.update();

        if(intaking){
            leftIntake.setPower(-0.9);
            rightIntake.setPower(0.9);
        }
        else{
            leftIntake.setPower(0.9);
            rightIntake.setPower(-0.9);
        }*/

    }
}





