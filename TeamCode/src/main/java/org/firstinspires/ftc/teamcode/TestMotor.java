package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestMotor extends OpMode {
    SampleDrive drive;
    Intake intake;

    public void init() {
        drive = new SampleDrive(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public void loop() {
        intake.testMotor(-0.833333333333);
    }
}
