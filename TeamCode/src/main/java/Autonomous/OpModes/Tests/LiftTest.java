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

import Actions.Annie.StoneStackingSystemV3;
import Actions.HardwareWrappers.DoubledSpoolMotor;

@Autonomous(name="LiftTest", group="Testers")
@Disabled
public class LiftTest extends LinearOpMode {
    // create objects and locally global variables here
    DoubledSpoolMotor lift;
    StoneStackingSystemV3 sss;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sss = new StoneStackingSystemV3(hardwareMap);
        lift = new DoubledSpoolMotor(new String[] {"liftMotor1", "liftMotor2"}, "ActionConfig/SSSLift.json", 50, 50, hardwareMap);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        int liftPosition = 0;
        boolean manualMode = false;
        while(opModeIsActive()){
            if(manualMode) {
                if (gamepad1.right_trigger > 0.1) lift.extendWithPower();
                else if (gamepad1.right_bumper) lift.retractWithPower();
                else lift.holdPosition();
            }else {
                if (gamepad1.a) {
                    liftPosition++;
                    while (gamepad1.a) ;
                    sss.liftToPosition(liftPosition);
                } else if (gamepad1.b) {
                    liftPosition--;
                    while (gamepad1.b) ;
                    sss.liftToPosition(liftPosition);
                }

                if (liftPosition > 4) liftPosition = 4;
                if (liftPosition < 0) liftPosition = 0;
            }

            if(gamepad1.x){
                manualMode = !manualMode;
                while (gamepad1.x);
            }
            telemetry.addData("M0", sss.getLiftPositionTicks(0));
            telemetry.addData("M1", sss.getLiftPositionTicks(1));
            telemetry.addData("liftPosition", liftPosition);
            telemetry.update();

        }
//        VuforiaHelper.kill(); -- this crashes the app...

        // finish drive code and test
        // may be a good idea to square self against wall

    }
    // misc functions here
}
