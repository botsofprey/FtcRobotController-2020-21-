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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.Location;
import DriveEngine.Annie.AnnieNavigation;

@Autonomous(name="AccelTest", group="Testers")
//@Disabled
public class AccelTest extends LinearOpMode {
    // create objects and locally global variables here
    AnnieNavigation robot;
    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        try {
            robot = new AnnieNavigation(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        // should only be used for a time keeper or other small things, avoid using this space when possible
        if(opModeIsActive()) robot.driveDistanceAccelerationBased(45,0,25,this);
//        if(opModeIsActive()) robot.driveDistance(15, 90, 25, this);
//        THIS IS FOR GRABBING FOUNDATION
//        robot.driveDistance(10, 270, 15, this);
//        robot.driveDistance(30, 270, 15, this);
//        if(opModeIsActive())sss.extendRightArm();
//        sleep();
//        sss.pauseRightArm();
//        if(opModeIsActive())sss.extendLeftArm();
//        if(opModeIsActive())sss.setLeftArmPosition(180);
//        if(opModeIsActive())sss.setRightArmPosition(180);
//        robot.driveDistance(38.42, 336, 15, this);

        // For future implementation: get stones
//        int reps = 6;
//        for (int i = 0; i < reps && opModeIsActive(); i++){
//            robot.driveDistance(20,0,25,this);
//            robot.driveDistance(87 + i * 8,90,25,this);
//            robot.driveDistance(24,180,25,this);
//            robot.driveDistance(89 + i * 8, 270, 25,this);
//        }

        robot.stopNavigation();

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
