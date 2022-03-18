package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class HardwareOdometry {

    //Motors
    public DcMotorEx FRMotor;
    public DcMotorEx FLMotor;
    public DcMotorEx BRMotor;
    public DcMotorEx BLMotor;

    //Odometers
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    //Servos
    public Servo randomServo;

    //Additional
    HardwareMap hardwareMap;
    public ElapsedTime pathTimer = new ElapsedTime();



    // constants to define robot geometry
    final static double L = 0.0;    // distance between encoder 1 and 2 in cm
    final static double B = 0.0;    // distance between the midpoint of encoder 1 and 2 and encoder 3
    final static double R = 0.0;    // wheel radius in cm
    final static double N = 0.0;    // encoder ticks per revolution, REV encoder
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    //to keep track of odometry encoders between updates
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;

    public double x = 0;
    public double y = 0;
    public double h = 0;

    private BNO055IMU imu;
    private double offset = 0.0;


    public HardwareOdometry(HardwareMap hwMap) {
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

        //shadow motors with the odometry encoders
        encoderLeft = BLMotor;
        encoderRight = BRMotor;
        encoderAux = FRMotor;

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void drive(double x, double y) {
        FRMotor.setPower(-x - y);
        FLMotor.setPower(-x + y);
        BRMotor.setPower(-x - y);
        BLMotor.setPower(-x + y);
    }

    public void fieldCentricDrive(double x, double y) { //working on...
        FRMotor.setPower(-x - y);
        FLMotor.setPower(-x + y);
        BRMotor.setPower(-x - y);
        BLMotor.setPower(-x + y);
    }

    public void odometry() { //called in teleop loop
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = -encoderAux.getCurrentPosition();

        //differences between positions
        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        //difference in x, y, and theta
        double dtheta = cm_per_tick * (dn2 - dn1) / L;
        double dx = cm_per_tick * (dn1 + dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L);

        double theta = h + (dtheta / 2.0);
        x += dx * Math.cos(theta) - dy * Math.sin(theta);
        y += dx * Math.sin(theta) + dy * Math.cos(theta);
        h += dtheta;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
    }
}
