package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AutonomousMecanumBlue2 extends LinearOpMode {
    SampleDrive drive;
    Catapult catapult;
    Carousel carousel;
    Intake intake;
    Lift lift;
    SensorREV2MDistance distance_sensor;

    //Three Front 2 meter distance sensors
    private DistanceSensor sensorDistanceL; //left front sensor
    private ModernRoboticsI2cRangeSensor sensorRangeM; //middle front range sensor
    private DistanceSensor sensorDistanceR; //right front sensor
    BNO055IMU imu;
    double offsetY;
    int barcodeLocation = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        drive = new SampleDrive(hardwareMap);
        catapult = new Catapult(hardwareMap);
        carousel = new Carousel(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
        sleep(1000);

        waitForStart();
        offsetY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // DETECTING TEAM SHIPPING ELEMENT (0 SECONDS)
        if (sensorDistanceL.getDistance(DistanceUnit.INCH) < 100 && sensorDistanceL.getDistance(DistanceUnit.INCH) < sensorDistanceR.getDistance(DistanceUnit.INCH)) {
            barcodeLocation = 1;
        } else if (sensorDistanceR.getDistance(DistanceUnit.INCH) < 100) {
            barcodeLocation = 2;
        } else {
            barcodeLocation = 3;
        }
        telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Right Sensor Range", String.format("%.01f in", sensorDistanceR.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Barcode Location", barcodeLocation);
        telemetry.update();

        //barcodeLocation = 1;
        //barcodeLocation = 2;
        //barcodeLocation = 3;

        // PLACING PRE-LOAD BOX (7 SECONDS)
        catapult.flapClose(); //secure pre-load box
        if (barcodeLocation == 1) { //drive to alliance shipping hub depending on barcode location
            drive.setPos(0, -400, 0, telemetry);
        } else if (barcodeLocation == 2) {
            drive.setPos(0, -550, 0, telemetry);
        } else {
            drive.setPos(0, -700, 0, telemetry);
        }
        drive.setPos(0, 0, -500, telemetry);
        intake.dropIntake(); //drop surgical tubing down
        sleep(1000);

        double time = getRuntime();
        while(getRuntime() < 2 + time) { //set catapult position depending on barcode location
            if(barcodeLocation == 1) {
                catapult.lower();
            } else if (barcodeLocation == 2) {
                catapult.middle();
            } else {
                catapult.upper();
            }
        }
        catapult.headUnfold(); //unfold catapult head
        sleep(1000);
        catapult.flapOpen(); //open up flap for outtake
        sleep(1000);
        catapult.returnPosition(); //return catapult to starting position

        // DELIVER DUCK THROUGH CAROUSEL (5 SECONDS)
        drive.setPos(0, 0, 1500, telemetry); // drive to carousel from alliance shipping hub
        drive.setPos(-4000, 0, 0, telemetry);
        drive.setPos(0, -2000, 0, telemetry);
        carousel.spinBlue(); //rotate carousel
        sleep(5000);
        carousel.spinOff();

        // COMPLETELY PARK IN WAREHOUSE (4 SECONDS)
        drive.setPos(-500, 0, 0, telemetry);
        drive.setPos(0,9000,0, telemetry); //drive to warehouse from carousel

        // COMPLETELY PARK IN STORAGE UNIT (1 SECOND)
        //drive.setPos(500, -500, 0, telemetry);
    }

    public void orient() {
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY < -5 ||
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY > 5) {
            drive.setPos(0, 0, (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY) * 10, telemetry);
            sleep(1);
        }

    }

}