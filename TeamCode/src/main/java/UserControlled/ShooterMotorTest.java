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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.Annie.MiscellaneousActions;
import Actions.Ultimate.ShooterSystemV2Test;

@TeleOp(name="Shooter RPM Test", group="Testers")
//@Disabled
public class ShooterMotorTest extends LinearOpMode {
    ShooterSystemV2Test shooter;
    int targetRPM = 3915;
    boolean dpadUpPressed = false, dpadDownPressed = false, triggerPressed = false, index = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSystemV2Test(hardwareMap);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.dpad_up && !dpadUpPressed) {
                dpadUpPressed = true;
                targetRPM += 5;
            } else if(!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            if(gamepad1.dpad_down && !dpadDownPressed) {
                dpadDownPressed = true;
                targetRPM -= 5;
            } else if(!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if(gamepad1.right_trigger > 0.1 && !triggerPressed) {
                triggerPressed = true;
                index = !index;
            } else if(gamepad1.right_trigger <= 0.1) {
                triggerPressed = false;
            }

            if(index) shooter.setIndexRight();
            else shooter.setIndexLeft();

            shooter.setRPM(targetRPM);
//            shooter.setHighGoalPower();
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.addData("Shooter PID", shooter.getPID().toString());
            telemetry.update();
        }
        shooter.kill();
    }
}
