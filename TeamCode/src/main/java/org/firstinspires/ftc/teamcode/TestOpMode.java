package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
Player 1 (gamepad1)
right stick     (hold)      drive
left bumper     (toggle)    team shipping element lift motor
right bumper    (toggle)    surgical tubing
*/

@TeleOp
public class TestOpMode extends OpMode {
    SampleDrive drive;
    Intake intake;
    Lift lift;

    private boolean lastPressedSurgical = false;
    private boolean surgicalToggle = false;
    private boolean lastPressedLiftMotor = false;
    private boolean liftMotorToggle = false;

    @Override
    public void init() {
        drive = new SampleDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
        drive.telemetry(telemetry);
        intake.telemetry(telemetry);
        lift.telemetry(telemetry);
        telemetry.update();

        drive.drive(gamepad1.left_stick_x, gamepad1.right_stick_y); //drive function

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

    }
}
