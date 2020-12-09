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

package UserControlled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Autonomous.ColorDetector;
import Autonomous.VuforiaHelper;

@TeleOp(name="Vuforia Test", group="Testers")
//@Disabled
public class VuforiaTest extends LinearOpMode {
    // create objects and locally global variables here

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        VuforiaHelper vuforia = new VuforiaHelper(hardwareMap);
        ColorDetector ringDetector = ColorDetector.ringDetector(vuforia);
        GamepadController controller = new GamepadController(gamepad1);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        // should only be used for a time keeper or other small things, avoid using this space when possible
        while (opModeIsActive()) {
            // main code goes here
            telemetry.addData("Orange pixels", "" + ringDetector.findNumDesiredPixels());
            telemetry.addData("R", "" + ringDetector.targetR);
            telemetry.addData("G", "" + ringDetector.targetG);
            telemetry.addData("B", "" + ringDetector.targetB);
            telemetry.addData("Tolerance", "" + ringDetector.tolerance);
            telemetry.addData("Num Rings", "" + ringDetector.getNumRingsFound(1));

            controller.update(gamepad1);

            if (controller.aHeld()) ringDetector.targetR++;
            if (controller.bHeld()) ringDetector.targetR--;
            if (controller.dpadUpHeld()) ringDetector.targetG++;
            if (controller.dpadDownHeld()) ringDetector.targetG--;
            if (controller.leftBumperHeld()) ringDetector.targetB++;
            if (controller.rightBumperHeld()) ringDetector.targetB--;
            if (controller.xHeld()) ringDetector.tolerance++;
            if (controller.yHeld()) ringDetector.tolerance--;

            telemetry.update();
        }
        // disable/kill/stop objects here
    }
    // misc functions here
}
