/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.OpModes.UltimateAuto.UltimateV2Autonomous;

import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_RIGHT;

/*
    An opmode for testing the robot's ability to drive in a square
 */
@Autonomous(name="UltimateAutoTestClass", group="test")  // @Autonomous(...) is the other common choice
//@Disabled
public class UltimateAutoTestClass extends LinearOpMode {

    @Override
    public void runOpMode() {

        // initialize robot // TODO: check starting location
        UltimateV2Autonomous robot = new UltimateV2Autonomous(AutoAlliance.RED, new Location(5, -5, 0), this);

        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.shooter.setRPM(1000);

        while (opModeIsActive()) {
            robot.powerShotIndex();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive());
        robot.kill();
    }
}
