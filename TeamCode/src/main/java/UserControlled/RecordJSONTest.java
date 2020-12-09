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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.json.JSONException;
import org.json.JSONObject;

@TeleOp(name="Record JSON", group="Competition")
//@Disabled
public class RecordJSONTest extends LinearOpMode {
    // create objects and locally global variables here

    public static final int TIMES_PER_SECOND = 10;

    DcMotor left, right;
    JoystickHandler jLeft, jRight;
    JSONObject object;

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "leftMotor");
        right = hardwareMap.get(DcMotor.class, "rightMotor");

        jLeft = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        jRight = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);

        object = new JSONObject();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // record data 10 times a second (to start)
        long targetTime = 1000000000 / TIMES_PER_SECOND; // this value should be used in nano's
        long elapsedTime;
        long startTime;

        try {

            // put the target time in the jsonObject since it might change between tests
            object.put("targetTime", targetTime);

            while (opModeIsActive()) {

                startTime = System.nanoTime();

                double leftPower = jLeft.y() + jRight.x();
                double rightPower = jLeft.y() - jRight.x();

                // record powers
                object.accumulate("left", leftPower);
                object.accumulate("right", rightPower);

                left.setPower(leftPower);
                right.setPower(rightPower);

                // convert time to milliseconds and sleep for the leftover time
                elapsedTime = startTime - System.nanoTime();
                sleep((targetTime - elapsedTime) / 1000000);
            }

        } catch(JSONException e) {
            telemetry.addData("Error", e.toString());
            while(opModeIsActive());
        }

        // Log the final values
        Log.d("Object", object.toString());

    }
    // misc functions here
}
