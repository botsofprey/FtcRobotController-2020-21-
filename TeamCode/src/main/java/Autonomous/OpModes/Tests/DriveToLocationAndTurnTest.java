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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import DriveEngine.Ultimate.UltimateNavigation2;

import Autonomous.Location;

import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_RIGHT;

@Autonomous(name="Drive To Location w/ Turn", group ="Testers")
//@Disabled
public class DriveToLocationAndTurnTest extends LinearOpMode {

    UltimateNavigation2 robot;

    @Override
    public void runOpMode() {

        try {
            robot = new UltimateNavigation2(hardwareMap, new Location(0, 0, 0), "RobotConfig/UltimateV2.json");
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        telemetry.addData("Status", "Initialized!");
        telemetry.update();
        waitForStart();

        robot.driveToLocationAndTurn(new Location(12, 12, 90), 25, this);
        Log.d("Final Robot Location", robot.getRobotLocation().toString());
        telemetry.addData("Location", robot.getRobotLocation().toString());
        telemetry.update();

        while (opModeIsActive());
        robot.stopNavigation();
    }
}
