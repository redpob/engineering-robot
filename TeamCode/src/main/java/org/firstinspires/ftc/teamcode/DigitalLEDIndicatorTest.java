package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DigitalLEDIndicatorTest extends LinearOpMode{
    LED red;
    LED green;
    public void runOpMode(){
        red = hardwareMap.get(LED.class, "red");
        green = hardwareMap.get(LED.class, "green");
                //light = hardwareMap.get
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("opMode is running");
            red.enableLight(true);
            telemetry.update();
        }
    }
}
