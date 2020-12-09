/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import Actions.Annie.StoneStackingSystemV2;
import Autonomous.Location;
import Autonomous.VisionHelperSkyStone;
import DriveEngine.Annie.AnnieNavigation;
import SensorHandlers.LIDARSensor;
import SensorHandlers.LimitSwitch;
import SensorHandlers.SensorPackage;

import static DriveEngine.Annie.AnnieNavigation.BACK_LEFT_HOLONOMIC_DRIVE_MOTOR;
import static DriveEngine.Annie.AnnieNavigation.BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR;
import static DriveEngine.Annie.AnnieNavigation.FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR;
import static DriveEngine.Annie.AnnieNavigation.FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR;

@Autonomous(name = "Self Wiring Test", group = "Testers")
@Disabled
public class SelfWiringTest extends LinearOpMode {
    private AnnieNavigation navigation;
    private StoneStackingSystemV2 sss;
    private SensorPackage sensors;

    private VisionHelperSkyStone robotVision;
    private int driveMotorCount[] = {0, 0, 0, 0};
    private int liftMotorCount = 0, leftArmCount = 0, rightArmCount = 0;
    private boolean liftSwitchGood = false, leftLIDARGood = false, rightLIDARGood = false,
            tflowDetectionGood = false, vuforiaNavigationGood = false, backLIDARGood = false,
            stoneDetectionGood = false, capstoneDeployGood = false, foundationGrippersGood = false;

    @Override
    public void runOpMode() {
        robotVision = new VisionHelperSkyStone(VisionHelperSkyStone.WEBCAM, hardwareMap);
        sss = new StoneStackingSystemV2(hardwareMap);
        sensors = new SensorPackage(new LimitSwitch(hardwareMap.get(TouchSensor.class, "liftReset"), "liftReset"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "left"), "left"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "back"), "back"),
                new LIDARSensor(hardwareMap.get(DistanceSensor.class, "right"), "right"));

        try {
            navigation = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        /** Wait for the game to begin */
        telemetry.addData("Status", "Initialized!");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running...");
        telemetry.update();

        checkDriveMotors();
        checkCamera();
        checkOtherSystems();

        reportRobotStatus();
        while (opModeIsActive());
        navigation.stopNavigation();
        robotVision.kill();
        sensors.kill();
        sss.kill();
    }

    private void checkDriveMotors() {
        telemetry.addData("Drive Motor", "Checking... Front Left");
        telemetry.update();
        navigation.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(5);
        sleep(200);
        navigation.brake();
        long tick = navigation.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        if(Math.abs(tick) > 30) driveMotorCount[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]++;
        navigation.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(-5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick() - tick) > 30) driveMotorCount[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]++;

        telemetry.addData("Drive Motor", "Checking... Front Right");
        telemetry.update();
        navigation.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick()) > 30) driveMotorCount[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]++;
        tick = navigation.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        navigation.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(-5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick() - tick) > 30) driveMotorCount[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]++;

        telemetry.addData("Drive Motor", "Checking... Back Left");
        telemetry.update();
        navigation.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick()) > 30) driveMotorCount[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]++;
        tick = navigation.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        navigation.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(-5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick() - tick) > 30) driveMotorCount[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]++;

        telemetry.addData("Drive Motor", "Checking... Back Right");
        telemetry.update();
        navigation.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick()) > 30) driveMotorCount[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]++;
        tick = navigation.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick();
        navigation.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(-5);
        sleep(200);
        navigation.brake();
        if(Math.abs(navigation.driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].getCurrentTick() - tick) > 30) driveMotorCount[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]++;
    }

    private void checkCamera() {
        telemetry.addData("Robot Vision", "Checking... TensorFlow");
        telemetry.addData("TensorFlow", "Put stones and skystones into the camera view");
        telemetry.update();
        sleep(1000);
        if(robotVision.getStonesInView().length > 0) tflowDetectionGood = true;

        telemetry.addData("Robot Vision", "Checking... Vuforia");
        telemetry.addData("Vuforia", "Put a navigation target into the camera view");
        telemetry.update();
        robotVision.startTrackingLocation();
        robotVision.startDetection();
        sleep(1000);
        if(robotVision.getRobotLocation() != null) vuforiaNavigationGood = true;

        telemetry.addData("Robot Vision", "Checking... Stone Detection");
        telemetry.addData("Stone Detection", "Put a skystone and a stone into the camera view");
        telemetry.update();
        robotVision.startSkyStoneDetection();
        sleep(1000);
        if(robotVision.getSkystoneLocation() != null) stoneDetectionGood = true;
        robotVision.stopDetection();
    }

    private void checkOtherSystems() {
        telemetry.addData("Sensors", "Checking... Left Distance");
        telemetry.update();
        sleep(300);
        telemetry.addData("Sensors", "Put your hand within 6 in of the LEFT distance sensor");
        telemetry.update();
        sleep(500);
        if(sensors.getSensor(LIDARSensor.class, "left").getDistance() <= 6) leftLIDARGood = true;

        telemetry.addData("Sensors", "Checking... Right Distance");
        telemetry.update();
        sleep(300);
        telemetry.addData("Sensors", "Put your hand within 6 in of the RIGHT distance sensor");
        telemetry.update();
        sleep(500);
        if(sensors.getSensor(LIDARSensor.class, "right").getDistance() <= 6) rightLIDARGood = true;

        telemetry.addData("Sensors", "Checking... Back Distance");
        telemetry.update();
        sleep(300);
        telemetry.addData("Sensors", "Put your hand within 6 in of the BACK distance sensor");
        telemetry.update();
        sleep(500);
        if(sensors.getSensor(LIDARSensor.class, "back").getDistance() <= 6) backLIDARGood = true;

        telemetry.addData("SSS", "Checking... Lift and Lift Limit Switch");
        telemetry.update();
        if(sensors.getSensor(LimitSwitch.class, "liftReset").isPressed()) liftSwitchGood = true;
        sss.liftStones();
        sleep(200);
        sss.pauseStoneLift();
        if(Math.abs(sss.getLiftPositionTicks()) > 30) liftMotorCount++;
        long tick = sss.getLiftPositionTicks();
        if(sensors.getSensor(LimitSwitch.class, "liftReset").isPressed()) liftSwitchGood = false;
        sss.lowerStones();
        sleep(200);
        sss.pauseStoneLift();
        if(Math.abs(sss.getLiftPositionTicks() - tick) > 30) liftMotorCount++;

        telemetry.addData("SSS", "Checking... Left Arm");
        telemetry.update();
        sss.extendLeftArm();
        sleep(200);
        sss.pauseLeftArm();
        if(Math.abs(sss.getLeftArmTick()) > 30) leftArmCount++;
        tick = sss.getLeftArmTick();
        sss.retractLeftArm();
        sleep(200);
        sss.pauseLeftArm();
        if(Math.abs(sss.getLeftArmTick() - tick) > 30) leftArmCount++;

        telemetry.addData("SSS", "Checking... Right Arm");
        telemetry.update();
        sss.extendRightArm();
        sleep(200);
        sss.pauseRightArm();
        if(Math.abs(sss.getRightArmTick()) > 30) rightArmCount++;
        tick = sss.getRightArmTick();
        sss.retractRightArm();
        sleep(200);
        sss.pauseRightArm();
        if(Math.abs(sss.getRightArmTick() - tick) > 30) rightArmCount++;
    }

    private void reportRobotStatus() {
        // Drive Motors
        if(driveMotorCount[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] == 2) {
            telemetry.addData("Drive Motor", "FL - Good! 2/2");
        } else if(driveMotorCount[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] == 1) {
            telemetry.addData("Drive Motor", "FL - Error! 1/2");
        } else telemetry.addData("Drive Motor", "FL - Error! 0/2");
        if(driveMotorCount[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] == 2) {
            telemetry.addData("Drive Motor", "FR - Good! 2/2");
        } else if(driveMotorCount[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] == 1) {
            telemetry.addData("Drive Motor", "FR - Error! 1/2");
        } else telemetry.addData("Drive Motor", "FR - Error! 0/2");
        if(driveMotorCount[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] == 2) {
            telemetry.addData("Drive Motor", "BL - Good! 2/2");
        } else if(driveMotorCount[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] == 1) {
            telemetry.addData("Drive Motor", "BL - Error! 1/2");
        } else telemetry.addData("Drive Motor", "BL - Error! 0/2");
        if(driveMotorCount[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] == 2) {
            telemetry.addData("Drive Motor", "BR - Good! 2/2");
        } else if(driveMotorCount[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] == 1) {
            telemetry.addData("Drive Motor", "BR - Error! 1/2");
        } else telemetry.addData("Drive Motor", "BR - Error! 0/2");
        telemetry.update();
        sleep(3000);

        // Stone Stacking System
        if(liftMotorCount == 2) telemetry.addData("SSS Lift", "Good! 2/2");
        else if(liftMotorCount == 1) telemetry.addData("SSS Lift", "Error! 1/2");
        else telemetry.addData("SSS Lift", "Error! 0/2");
        if(leftArmCount == 2) telemetry.addData("SSS L Arm", "Good! 2/2");
        else if(leftArmCount == 1) telemetry.addData("SSS L Arm", "Error! 1/2");
        else telemetry.addData("SSS L Arm", "Error! 0/2");
        if(rightArmCount == 2) telemetry.addData("SSS R Arm", "Good! 2/2");
        else if(rightArmCount == 1) telemetry.addData("SSS R Arm", "Error! 1/2");
        else telemetry.addData("SSS R Arm", "Error! 0/2");
        if(capstoneDeployGood) telemetry.addData("SSS Capstone Deploy", "Good!");
        else telemetry.addData("SSS Capstone Deploy", "Bad!");
        if(foundationGrippersGood) telemetry.addData("SSS Foundation Grippers", "Good!");
        else telemetry.addData("SSS Foundation Grippers", "Bad!");
        telemetry.update();
        sleep(3000);

        // Sensors
        if(liftSwitchGood) telemetry.addData("Sensor Package Lift Reset", "Good!");
        else telemetry.addData("Sensor Package Lift Reset", "Bad!");
        if(leftLIDARGood) telemetry.addData("Sensor Package Left Distance", "Good!");
        else telemetry.addData("Sensor Package Left Distance", "Bad!");
        if(rightLIDARGood) telemetry.addData("Sensor Package Right Distance", "Good!");
        else telemetry.addData("Sensor Package Right Distance", "Bad!");
        if(backLIDARGood) telemetry.addData("Sensor Package Back Distance", "Good!");
        else telemetry.addData("Sensor Package Back Distance", "Bad!");
        telemetry.update();
        sleep(3000);

        // Camera
        if(tflowDetectionGood) telemetry.addData("Robot Vision", "TensorFlow - Good!");
        else telemetry.addData("Robot Vision", "TesnsorFlow - Bad!");
        if(vuforiaNavigationGood) telemetry.addData("Robot Vision", "Vuforia - Good!");
        else telemetry.addData("Robot Vision", "Vuforia - Bad!");
        if(stoneDetectionGood) telemetry.addData("Robot Vision", "Stone Processor - Good!");
        else telemetry.addData("Robot Vision", "Stone Processor - Bad!");
        telemetry.update();
        sleep(3000);

        // Misc

    }
}
