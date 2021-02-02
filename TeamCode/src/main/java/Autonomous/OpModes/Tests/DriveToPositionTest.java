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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.ArrayList;
import java.util.List;

import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.OpModes.AnnieAuto.AutonomousRoutines;
import DriveEngine.Annie.AnnieNavigation;

@Autonomous(name="Drive to Position", group="Testers")
@Disabled
public class DriveToPositionTest extends LinearOpMode {

    double heading = 270;
    AnnieNavigation robot;
    DistanceSensor back;

    @Override
    public void runOpMode() {
        try {
            robot = new AnnieNavigation(hardwareMap, new Location(60, -32), heading, "RobotConfig/AnnieV1.json");
//            robot.disableSensorLocationTracking();
        } catch (Exception e) {
            e.printStackTrace();
        }
        back = hardwareMap.get(DistanceSensor.class, "back");

        telemetry.addData("Start Location", robot.getRobotLocation());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Location redQuarryPosition1 = new Location(-33.5, -26.5, heading);
        Location redFoundationCenterAtStart = new Location(-33.5, 50, heading);
        Location redBuildSiteNearWall = new Location(-60, 50, heading);
        Location redQuarryNearWall4 = new Location(-60, -50.5, heading);
        Location redQuarryPosition4 = new Location(-33.5, -50.5, heading);
        Location redUnderBridgeCenter = new Location(-47.5, 0.0, heading);

        List<Location> locations = new ArrayList<Location>();
        locations.add(redQuarryPosition1);
        locations.add(redFoundationCenterAtStart);
        locations.add(redBuildSiteNearWall);
        locations.add(redQuarryNearWall4);
        locations.add(redQuarryPosition4);
        locations.add(redFoundationCenterAtStart);
        locations.add(redBuildSiteNearWall);
        locations.add(redQuarryNearWall4);
        locations.add(redUnderBridgeCenter);

        telemetry.addData("Start Location 2", robot.getRobotLocation());
        telemetry.update();
        //robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_LEFT_RED, 45, this);
//        robot.navigatePathPID(locations.toArray(new Location[0]), 15, this);
/*
        Location QUARRY_CENTRAL = new Location(35, 55, heading);
        Location FOUNDATION_CENTRAL = new Location(35, -55, heading);

        for (int i = 0; i < 10 && opModeIsActive(); ++i) {
            robot.driveToLocationPID(QUARRY_CENTRAL, 25, 0.5, 30.0, this);
            sleep(2000);
            robot.driveToLocationPID(FOUNDATION_CENTRAL, 25, 0.5, 30.0, this);
            sleep(2000);
        }
*/
        AutonomousRoutines.runGrabTest(AutoAlliance.RED, this);

//        for (int i = 0; i < 10 && opModeIsActive(); ++i) {
//            robot.driveToLocationPID(new Location(55, -34.5, 270), 25, 0.5, 30, this);
//            sleep(2000);
//            robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_RIGHT_RED, 25, 0.5, 30, this);
//            sleep(2000);
//            robot.driveToLocationPID(new Location(55, -34.5, 270), 25, 0.5, 30, this);
//            sleep(2000);
//            robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_CENTER_RED, 25, 0.5, 30, this);
//            sleep(2000);
//            robot.driveToLocationPID(new Location(55, -34.5, 270), 25, 0.5, 30, this);
//            sleep(2000);
//            robot.driveToLocationPID(ConfigVariables.SECOND_STONE_GROUP_LEFT_RED, 25, 0.5, 30, this);
//            sleep(2000);
//        }


        telemetry.addData("End Location", robot.getRobotLocation());
        telemetry.update();

        robot.stopNavigation();

    }
}
