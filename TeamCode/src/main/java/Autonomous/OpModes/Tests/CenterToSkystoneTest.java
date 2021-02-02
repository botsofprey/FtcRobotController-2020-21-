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

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;

import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.Location;
import Autonomous.VuforiaHelper;
import DriveEngine.Annie.AnnieNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="CenterToSkystone Test", group="Testers")
@Disabled
public class CenterToSkystoneTest extends LinearOpMode {
    // create objects and locally global variables here
    SkystoneImageProcessor stoneFinder;
    VuforiaHelper vuforia;
    AnnieNavigation robot;
    DistanceSensor back, right, left;

    @Override
    public void runOpMode() {

        // initialize objects and variables here
        vuforia = new VuforiaHelper(hardwareMap);
//        for (int i = 0; i < 3; i++)
//            (new DistanceSensor[] { back, right, left })[i] = hardwareMap.get(DistanceSensor.class, (new String[] { "back", "right", "left"})[i]);
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        stoneFinder = new SkystoneImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH,.1,1, SkystoneImageProcessor.STONE_COLOR.BLACK);
        try {
            robot = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }
        // also create and initialize function local variables here

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
//        turn((clockwise) ? -rps : rps);
        robot.driveDistance(15, AnnieNavigation.FORWARD, 20, this);
        // rough center...
        boolean skystoneFound = false;
        while (opModeIsActive() && right.getDistance(INCH) > 5.5) {
            if(!skystoneFound) robot.driveOnHeadingPID(AnnieNavigation.RIGHT,10, this);  //NOTE: MOVING RIGHT
            Bitmap bmp = null;
            ArrayList<Integer> blockCenters;
            long timeStart = System.currentTimeMillis();
            while (bmp == null && System.currentTimeMillis() - timeStart < 50) {
                bmp = vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
            }
            if(bmp != null){
                blockCenters = stoneFinder.findColumns(bmp, false);

                if (blockCenters.size() > 0) {
//                telemetry.addData("Skystone", "" + blockCenters.get(0));
                    skystoneFound = true;
                    if (robot.centerOnSkystone(blockCenters.get((blockCenters.size() == 1) ? 0 : 1), 2, 20,this)) break; //Get left-most skystone
                } else skystoneFound = false;
            }
        }
        robot.brake();
        double distToWall = right.getDistance(INCH);

        telemetry.addData("dist to wall", distToWall);
        if (distToWall > 42) { // good
            while (opModeIsActive() && right.getDistance(INCH) > 41.50) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 20, this); // STONE 1 RIGHT
            telemetry.addData("first stone", "move right");
            telemetry.update();
        } else if (distToWall > 35.0) { // good
            while (opModeIsActive() && right.getDistance(INCH) < 38.50) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, this);// STONE 1 LEFT
            telemetry.addData("first stone", "move left");
            telemetry.update();
        } else if (distToWall > 32.5) { // good for now
            while (opModeIsActive() && right.getDistance(INCH) > 37.25) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 20, this);// STONE 2 RIGHT
            telemetry.addData("second stone", "move right");
            telemetry.update();
        } else if (distToWall > 28) { // keep testing
            while (opModeIsActive() && right.getDistance(INCH) < 30.00) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, this);// STONE 2 LEFT
            telemetry.addData("second stone", "move left");
            telemetry.update();
        } else if (distToWall > 24) { // keep testing
            while (opModeIsActive() && right.getDistance(INCH) > 24.50) robot.driveOnHeadingPID(AnnieNavigation.RIGHT, 20, this);// STONE 3 RIGHT
            telemetry.addData("third stone", "move right");
            telemetry.update();
        } else if (distToWall > 19.5) { // keep testing
            while (opModeIsActive() && right.getDistance(INCH) < 24.50) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 20, this);// STONE 3 LEFT
            telemetry.addData("third stone", "move left");
            telemetry.update();
        }
        robot.brake();

        while(opModeIsActive() && back.getDistance(INCH) < 28.0) {
            robot.driveOnHeadingPID(AnnieNavigation.FORWARD,15,0,this);
        }
        robot.brake();
        while (opModeIsActive());
        robot.stopNavigation(); // ALWAYS kill everything at the end, leads to crashes of the app otherwise
    }
}
