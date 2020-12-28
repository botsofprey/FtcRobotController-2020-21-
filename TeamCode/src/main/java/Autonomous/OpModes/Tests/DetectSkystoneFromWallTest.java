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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import Actions.Annie.MiscellaneousActionsV2;
import Actions.Annie.StoneStackingSystemV3;
import Autonomous.ConfigVariables;
import Autonomous.ImageProcessing.SkystoneImageProcessor;
import Autonomous.Location;
import Autonomous.VuforiaHelper;
import DriveEngine.Annie.AnnieNavigation;
import SensorHandlers.UltrasonicIRSensor;

@Autonomous(name="Detect Skystone From Wall", group="Competition")
//@Disabled
public class DetectSkystoneFromWallTest extends LinearOpMode {
    // create objects and locally global variables here
    AnnieNavigation robot;
    StoneStackingSystemV3 sss;
    VuforiaHelper vuforia;
    SkystoneImageProcessor stoneFinder;
    DistanceSensor back, right, left;
    UltrasonicIRSensor front;
    MiscellaneousActionsV2 otherActions;
    static final double LEFT_DIST = .48, BACK_DIST = .91, CENTER_DIST = .73, RIGHT_DIST = .91; //LEFT .54 LEAVE THIS COMMENT FOR NOW PLEASE
    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        vuforia = new VuforiaHelper(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        front = new UltrasonicIRSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front"), "front");
        stoneFinder = new SkystoneImageProcessor(SkystoneImageProcessor.DESIRED_HEIGHT, SkystoneImageProcessor.DESIRED_WIDTH,.1,1, SkystoneImageProcessor.STONE_COLOR.BLACK);
        otherActions = new MiscellaneousActionsV2(hardwareMap);

        try {
            robot = new AnnieNavigation(hardwareMap, new Location(60, -32, 270), 270, "RobotConfig/AnnieV1.json");
            robot.stopLoggingData();
//            robot.disableSensorLocationTracking();
        } catch (Exception e) {
            e.printStackTrace();
        }
        sss = new StoneStackingSystemV3(hardwareMap);

        vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT);
        sleep(100);
        // Stone detection
        int pos = SkystoneImageProcessor.UNKNOWN;
        while(!opModeIsActive()) {
            pos = stoneFinder.getSkystoneRelativePosition(vuforia.getImage(SkystoneImageProcessor.DESIRED_WIDTH, SkystoneImageProcessor.DESIRED_HEIGHT));
            String posStr = "";
            switch (pos) {
                case SkystoneImageProcessor.LEFT:
                    posStr = "LEFT";
                    break;
                case SkystoneImageProcessor.CENTER:
                    posStr = "CENTER";
                    break;
                case SkystoneImageProcessor.RIGHT:
                    posStr = "RIGHT";
                    break;
                default:
                    posStr = "NOT FOUND";
                    break;
            }
            telemetry.addData("Skystone position", posStr);
            // add any other useful telemetry data or logging data here
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Op Mode", opModeIsActive());
            telemetry.update();
        }
        // nothing goes between the above and below lines
        waitForStart();


        robot.startLoggingData();
        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_GRAB);
        sleep(500);
        if(opModeIsActive()) sss.setCentralGripperDegree(StoneStackingSystemV3.CENTRAL_ARM_RELEASE);
        sss.liftToPosition(1);

        final double STONE_DRIVE_SPEED = 25;
        final double LEAVE_STONE_QUARRY_OFFSET = 8.5;
        // having only two values and a default is better to use an if-else if-else block
        switch (pos) {
            // also using a tolerance of 1.5 works pretty well
            case SkystoneImageProcessor.CENTER:
                robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_CENTER_RED, STONE_DRIVE_SPEED, 1.5, this);
                break;
            case SkystoneImageProcessor.LEFT:
                robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_LEFT_RED, STONE_DRIVE_SPEED, this);
                break;
            default:
                robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_RIGHT_RED, STONE_DRIVE_SPEED, 1.5, this);
                break;
        }
        if(front.getDistance() < 2.0) {
            robot.driveDistance(3, 180, STONE_DRIVE_SPEED, this);
        }
        if(front.getDistance() > 2.75) {
            double measuredDistance = front.getDistance();
            sss.liftToPosition(0);
            robot.driveDistance(measuredDistance - 1.25, 0, 15, this);
        } else {
            sss.liftToPosition(0);
            sleep(250);
        }

        sss.grabStoneCenter();
        sleep(300);
        robot.driveToLocationPID(new Location(robot.getRobotLocation().getX()+LEAVE_STONE_QUARRY_OFFSET, robot.getRobotLocation().getY(), AnnieNavigation.LEFT), STONE_DRIVE_SPEED, this);
        sleep(150);
        robot.driveToLocationPID(ConfigVariables.UNDER_RED_BRIDGE, 45, this); // consider drive distance instead
        robot.driveDistance(10, 90, STONE_DRIVE_SPEED, this);
        sss.liftToPosition(1);
        robot.driveToLocationPID(ConfigVariables.RED_FOUNDATION_CENTER, 45, this);
//        sleep(2000);
//        robot.driveToLocationPID(ConfigVariables.RED_FOUNDATION_LEFT, 15, 0.3, 30.0, this);
//        sleep(2000);
        robot.turnController.setSp(robot.getOrientation());
        long startTime = System.currentTimeMillis();
        while(opModeIsActive() && front.getDistance() > 1.2 && System.currentTimeMillis() - startTime < 2000) {
            robot.driveOnHeadingPID(0, 15, this);
        }
        robot.brake();

        otherActions.grabFoundation();
        sleep(750);
        robot.driveOnHeadingWithTurning(200,0.5,.2);
        sleep(1750);
        robot.brake();
        robot.driveDistance(30, AnnieNavigation.FORWARD, STONE_DRIVE_SPEED, this);
        otherActions.releaseFoundation();
        sss.releaseStoneCenter();
        sleep(750);

        robot.driveToLocationPID(new Location(35, robot.getRobotLocation().getY() - 5, 0), 35, this);
        sss.liftToPosition(0);
        robot.driveToLocationPID(ConfigVariables.UNDER_RED_BRIDGE_0_HEADING, STONE_DRIVE_SPEED, this);
        robot.driveToLocationPID(new Location(35, -35, 0), 35, this);
        sss.liftToPosition(1);
        robot.turnToHeading(-90, this);

//        robot.driveToLocationPID(ConfigVariables.RED_FOUNDATION_LEFT, 20, this);
//        sss.liftToPosition(0);
//        sleep(150);
//        robot.driveToLocationPID(ConfigVariables.UNDER_RED_BRIDGE, 35, this);
//        sleep(150);
//        robot.driveToLocationPID(ConfigVariables.BEHIND_RED_QUARRY, 45, this);
//        sleep(20000);
        switch (pos) {
            case SkystoneImageProcessor.CENTER:
                robot.driveToLocationPID(ConfigVariables.FIRST_STONE_GROUP_CENTER_RED, STONE_DRIVE_SPEED, this);
                break;
            case SkystoneImageProcessor.LEFT:
                robot.driveToLocationPID(ConfigVariables.FIRST_STONE_GROUP_LEFT_RED, STONE_DRIVE_SPEED, this); // need to specialize for this position
                break;
            default:
                robot.driveToLocationPID(ConfigVariables.FIRST_STONE_GROUP_RIGHT_RED, STONE_DRIVE_SPEED, this);
                break;
        }
        if(front.getDistance() > 2.75) {
            double measuredDistance = front.getDistance();
            sss.liftToPosition(0);
            robot.driveDistance(measuredDistance - 1.25, 0, 15, this);
        } else {
            sss.liftToPosition(0);
            sleep(250);
        }

        sss.grabStoneCenter();
        sleep(300);
        robot.driveToLocationPID(new Location(robot.getRobotLocation().getX()+LEAVE_STONE_QUARRY_OFFSET, robot.getRobotLocation().getY(), robot.getOrientation()), STONE_DRIVE_SPEED, this);
        sleep(150);
        robot.turnToHeading(0, this);
        robot.driveToLocationPID(ConfigVariables.UNDER_RED_BRIDGE_BUILDING_ZONE, STONE_DRIVE_SPEED, this);
        sss.liftToPosition(2);
        robot.driveToLocationPID(new Location(35, 35, 0), 35, this);
//        robot.driveDistance(10, 90, STONE_DRIVE_SPEED, this);
        startTime = System.currentTimeMillis();
        robot.turnController.setSp(robot.getOrientation());
        while (opModeIsActive() && front.getDistance() > 1.2 && System.currentTimeMillis() - startTime < 2000) {
            robot.driveOnHeadingPID(0, 25, this);
        }
        robot.brake();
//        robot.driveToLocationPID(new Location(35, 71-20, 0), 25, this); // TODO fix this position
        sss.releaseStoneCenter();
        sleep(250);
        robot.driveToLocationPID(new Location(35, 35, 0), STONE_DRIVE_SPEED, this);
        sss.liftToPosition(0);
        robot.driveToLocationPID(ConfigVariables.UNDER_RED_BRIDGE_0_HEADING, 45, this);
//        // robot has issues staying on it's heading here... may need to add the PID control to the heading, would prefer not to
//        robot.driveToLocationPID(ConfigVariables.RED_FOUNDATION_LEFT, 45, this);
//        sleep(2000);
//        sss.liftToPosition(2);
//        robot.driveToLocationPID(ConfigVariables.RED_FOUNDATION_STACK_LEFT, 20, 0.3, 2.0, this);
//        sss.releaseStoneCenter();
//        sleep(300);
//        robot.driveToLocationPID(ConfigVariables.RED_FOUNDATION_LEFT, STONE_DRIVE_SPEED, this); //tolerance of .25
//        sss.liftToPosition(0);

        while (opModeIsActive());
        robot.stopNavigation();
        sss.kill();
//        VuforiaHelper.kill(); -- this crashes the app...

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
