/*
Copyright (c) 2018 FIRST
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorREV2MDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 * Control I2C 0
 */
@TeleOp(name = "REV 2M Distance Sensor", group = "Sensor")
//@Disabled
public class SensorREV2MDistance extends LinearOpMode {
    private DistanceSensor sensorRangeLeft;
    private DistanceSensor sensorRangeRight;
    int barcodeLocation = 0;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
            sensorRangeRight = hardwareMap.get(DistanceSensor.class, "sensor_range_right");
            // generic DistanceSensor methods.
            telemetry.addData("deviceName",sensorRangeLeft.getDeviceName() );
            telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorRangeLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("deviceName",sensorRangeRight.getDeviceName() );
            telemetry.addData("Right Sensor Range", String.format("%.01f in", sensorRangeRight.getDistance(DistanceUnit.INCH)));

            if (sensorRangeLeft.getDistance(DistanceUnit.INCH) < 100 && sensorRangeLeft.getDistance(DistanceUnit.INCH) < sensorRangeRight.getDistance(DistanceUnit.INCH)) {
                barcodeLocation = 0;
            } else if (sensorRangeRight.getDistance(DistanceUnit.INCH) < 100 && sensorRangeRight.getDistance(DistanceUnit.INCH) < sensorRangeLeft.getDistance(DistanceUnit.INCH)) {
                barcodeLocation = 1;
            } else {
                barcodeLocation = 2;
            }

            telemetry.addData("Barcode Location", barcodeLocation);
            telemetry.update();
        }
    }

}