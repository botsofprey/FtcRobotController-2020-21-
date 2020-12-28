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

package Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import Actions.Annie.MiscellaneousActionsV2;
import Actions.Annie.StoneStackingSystemV3;
import Autonomous.Location;
import DriveEngine.Annie.AnnieNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

@Autonomous(name="FoundationBlue", group="Competition")
//@Disabled
public class FoundationBlue extends LinearOpMode {
    // create objects and locally global variables here
    AnnieNavigation robot;
    DistanceSensor back, right, left;
    MiscellaneousActionsV2 otherActions;
    StoneStackingSystemV3 sss;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        back = hardwareMap.get(DistanceSensor.class, "back");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        otherActions = new MiscellaneousActionsV2(hardwareMap);
        sss = new StoneStackingSystemV3(hardwareMap);

        try {
            robot = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Stone detection


        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();



//        sss.setLiftPosition(5);
        while(left.getDistance(INCH) > 18) robot.driveOnHeadingPID(AnnieNavigation.LEFT, 25,this);
        robot.brake();
        while(back.getDistance(INCH) < 20) robot.driveOnHeadingPID(AnnieNavigation.FORWARD, 25,this);
        robot.brake();
        while(back.getDistance(INCH) < 36) robot.driveOnHeadingPID(AnnieNavigation.FORWARD, 15, this);
        robot.brake();
        otherActions.grabFoundation();
        sleep(750);
        robot.driveOnHeadingWithTurning(160,0.5,-.2);
        sleep(1750);
        robot.brake();
        otherActions.grabFoundation();
        sleep(500);
        robot.driveDistance(36, 0, 20, this);

        robot.stopNavigation();
//        VuforiaHelper.kill(); -- this crashes the app...

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
