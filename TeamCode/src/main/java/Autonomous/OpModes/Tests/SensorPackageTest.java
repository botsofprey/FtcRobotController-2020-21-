/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Autonomous.OpModes.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import SensorHandlers.LIDARSensor;
import SensorHandlers.MagneticLimitSwitch;
import SensorHandlers.SensorPackage;
import SensorHandlers.UltrasonicIRSensor;

@Autonomous(name="Sensor Package Test", group="Testers")
//@Disabled
public class SensorPackageTest extends LinearOpMode {
    // create objects and locally global variables here
    SensorPackage sensors;

    @Override
    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sensors = new SensorPackage(new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"),"left"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"),  "back"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "right"), "right"),
                /*new LimitSwitch(hardwareMap.get(TouchSensor.class, "liftReset"), "liftReset"),*/
                new UltrasonicIRSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic"), "ultrasonic"),
                new MagneticLimitSwitch(hardwareMap.get(DigitalChannel.class, "limit"), "limit")
                /*new LIDARSensor(hardwareMap.get(DistanceSensor.class, "front"), "front")*/);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        while (opModeIsActive()) {
//            telemetry.addData("Left Sensor", sensors.getSensor(LIDARSensor.class, "left"));
            telemetry.addData("Left Dist", sensors.getSensor(LIDARSensor.class, "left").getDistance());
//            telemetry.addData("Back Sensor", sensors.getSensor(LIDARSensor.class, "back"));
            telemetry.addData("Back Dist", sensors.getSensor(LIDARSensor.class, "back").getDistance());
            telemetry.addData("Right Dist", sensors.getSensor(LIDARSensor.class, "right").getDistance());
//            telemetry.addData("Lift Reset", sensors.getSensor(LimitSwitch.class, "liftReset").isPressed());
            telemetry.addData("Ultrasonic Dist", sensors.getSensor(UltrasonicIRSensor.class, "ultrasonic").getDistance());
            telemetry.addData("Magnetic state", sensors.getSensor(MagneticLimitSwitch.class, "limit").isActivated());
//            telemetry.addData("Front Dist", sensors.getSensor(LIDARSensor.class, "front").getDistance());
            telemetry.update();
        }

        sensors.kill();

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
