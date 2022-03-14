package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Player 1 (gamepad1)
right stick     (hold)      drive
left bumper     (toggle)    team shipping element lift motor
right bumper    (toggle)    surgical tubing
*/

@TeleOp
public class TestOpModeOdometry extends LinearOpMode {
    HardwareOdometry robot;
    Intake intake;
    Lift lift;

    private boolean lastPressedSurgical = false;
    private boolean surgicalToggle = false;
    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareOdometry(hardwareMap);

        ElapsedTime timer = new ElapsedTime();
        robot.pathTimer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while(opModeIsActive()) {
            robot.telemetry(telemetry);
            intake.telemetry(telemetry);
            lift.telemetry(telemetry);

            robot.drive(gamepad1.left_stick_x, gamepad1.right_stick_y); //drive function

            robot.odometry();

            telemetry.addData("LRA", "%6d    %6d    %6d", robot.currentLeftPosition, robot.currentRightPosition, robot.currentAuxPosition);
            telemetry.addData("xyh", "#6.1f cm   %6.1f cm   %6.1f deg", robot.x, robot.y, Math.toDegrees(robot.h));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());


            //TEAM SHIPPING ELEMENT LIFT
            if (gamepad2.left_bumper && !lastPressedLiftMotor) {
                liftMotorToggle = !liftMotorToggle;
            }
            if (liftMotorToggle) {
                lift.lift();
            } else {
                lift.lower();
            }
            lastPressedLiftMotor = gamepad2.left_bumper;

            //WHEEL INTAKE
            if (gamepad2.right_bumper && !lastPressedSurgical) {
                surgicalToggle = !surgicalToggle;
            }
            if (surgicalToggle) {
                intake.intakeOn();
            } else {
                intake.intakeOff();
            }
            lastPressedSurgical = gamepad1.right_bumper;

            telemetry.update();
            timer.reset();
        }
    }
}
