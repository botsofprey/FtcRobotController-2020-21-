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

package Autonomous.OpModes.AnnieAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Actions.Annie.StoneStackingSystemV2;
import Autonomous.Location;
import Autonomous.VisionHelperSkyStone;
import DriveEngine.Annie.AnnieNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="Red", group="Competition")
//@Disabled
public class AnnieV1AutoRed extends LinearOpMode {
    // create objects and locally global variables here
    AnnieNavigation robot;
    StoneStackingSystemV2 sss;
    VisionHelperSkyStone vision;
    DistanceSensor back, right, left;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        try {
            robot = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystemV2(hardwareMap);

        // Stone detection
        vision = new VisionHelperSkyStone(VisionHelperSkyStone.WEBCAM, VisionHelperSkyStone.LOCATION, hardwareMap);
        vision.startDetection();
        vision.startTrackingLocation();

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();

        Location skystoneLocation = vision.getSkystoneLocation();
        Orientation skystoneOrientation = vision.getSkystoneOrientation();

        telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
        telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
        telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
        telemetry.update();

//        Drive forward until 15 inches from wall
        double distToWall = back.getDistance(INCH);
        while (opModeIsActive() && distToWall < 10) {
            robot.driveOnHeadingPID(0, 10, 0, this);
            distToWall = back.getDistance(INCH);

            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
            telemetry.update();
        }
        robot.brake();

//        Search for skystone and break if found
        distToWall = left.getDistance(INCH);
        while (opModeIsActive() && distToWall > 6.25) {
            robot.driveOnHeadingPID(-90, 5, 0, this);
            skystoneLocation = vision.getSkystoneLocation();
            skystoneOrientation = vision.getSkystoneOrientation();
            if (skystoneLocation != null && skystoneOrientation != null) {
                break;
            }
            distToWall = left.getDistance(INCH);

            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
            telemetry.update();
        }
        robot.brake();

//        If we are at wall we didn't find skystone. Otherwise, move to the skystone
        if(distToWall <= 6.25) {
            telemetry.addData("Did not find skystone", "");
            telemetry.update();
        } else {
            double distToZero = Math.abs(vision.getSkystoneLocation().getY());
            while (opModeIsActive() && distToZero > 0.2) {
                robot.driveOnHeadingPID((vision.getSkystoneLocation().getY() > 0)? 90:-90, 15, 0, this);
                distToZero = Math.abs(vision.getSkystoneLocation().getY());

                telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
                telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
                telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
                telemetry.update();
            }
            robot.brake();
        }

//        Grab skystone
//        robot.driveDistance(13, 0, 15, this);
        distToWall = back.getDistance(INCH);
        while(opModeIsActive() && distToWall < 27.5){
            robot.driveOnHeadingPID(AnnieNavigation.FORWARD,5,0,this);
            distToWall = back.getDistance(INCH);
        }
        robot.brake();

        sss.grabStoneCenter();
        sleep(750); // wait to grab the stone TODO: grabStoneCenterAndWait() to wait until correct angle reached?
        distToWall = back.getDistance(INCH);
        while(opModeIsActive() && distToWall > 10) {
            robot.driveOnHeadingPID(180, 20, 0, this);
            distToWall = back.getDistance(INCH);
        }
        robot.brake();

//        Drive to foundation
        distToWall = right.getDistance(INCH);
        while (opModeIsActive() && distToWall > 32) {
            robot.driveOnHeadingPID(90, 15, 0, this);
            distToWall = right.getDistance(INCH);

            telemetry.addData("Left", Double.toString(left.getDistance(INCH)));
            telemetry.addData("Right", Double.toString(right.getDistance(INCH)));
            telemetry.addData("Back", Double.toString(back.getDistance(INCH)));
            telemetry.update();
        }
        robot.brake();

        // lift stone and stack
        if(opModeIsActive()) sss.setLiftPosition(3);

        // drive distance
        distToWall = back.getDistance(INCH);
        for(int i = 0; i < 2; i++) {
            while (opModeIsActive() && distToWall < 28.5) {
                robot.driveOnHeadingPID(0, 15, 0, this);
                distToWall = back.getDistance(INCH);
            }
            robot.brake();
            distToWall = back.getDistance(INCH);
        }
        while(opModeIsActive() && sss.getLiftPositionInches() < 2.9);
        if(opModeIsActive()) sss.setLiftPosition(2.5);
        while(opModeIsActive() && sss.getLiftPositionInches() > 2.55);

        if(opModeIsActive()) sss.releaseStoneCenter();

        while (opModeIsActive());

        vision.kill();
        robot.stopNavigation();
        sss.kill();

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
