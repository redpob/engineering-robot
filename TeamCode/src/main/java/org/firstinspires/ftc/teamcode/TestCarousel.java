package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestCarousel extends OpMode {
    Carousel carousel;

    @Override
    public void init() {
        carousel = new Carousel(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.x) {
            carousel.spinBlue();
        }
        if(gamepad1.b) {
            carousel.spinRed();
        }
        if(gamepad1.a) {
            carousel.spinOff();
        }
    }
}
