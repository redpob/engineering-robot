package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestServo extends OpMode {
    SampleDrive drive;
    Intake intake;
    Carousel carousel;

    public void init() {
        drive = new SampleDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        carousel = new Carousel(hardwareMap);
    }

    public void loop() {
        carousel.testServo(10);
    }
}
