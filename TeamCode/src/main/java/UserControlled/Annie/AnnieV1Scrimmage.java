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

package UserControlled.Annie;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.Annie.StoneStackingSystemV1;
import DriveEngine.HolonomicDriveSystemTesting;
import UserControlled.JoystickHandler;

@TeleOp(name="Annie Scrimmage", group="Competition")
//@Disabled
public class AnnieV1Scrimmage extends LinearOpMode {
    // create objects and locally global variables here
    HolonomicDriveSystemTesting robot;
    StoneStackingSystemV1 sss;
    JoystickHandler leftStick, rightStick;
    boolean eStop = false, leftArmMode = false, rightArmMode = false, bothArmMode = true, slowMode = false;
    boolean startReleased = true, eStopButtonsReleased = true;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        robot = new HolonomicDriveSystemTesting(hardwareMap, "RobotConfig/AnnieV1.json");
        sss = new StoneStackingSystemV1(hardwareMap);
        leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        // should only be used for a time keeper or other small things, avoid using this space when possible
        while (opModeIsActive()) {
            // main code goes here
            updateEStop();
            if(!eStop) {
                if (startReleased && gamepad1.start) {
                    startReleased = false;
                    slowMode = !slowMode;
                } else if (!gamepad1.start) {
                    startReleased = true;
                }

                updateEStop();
                controlDrive();

                updateEStop();
                controlStoneStackingSystem();
            }
            if(eStop) {
                stopActions();
            }
            // telemetry and logging data goes here
            telemetry.update();
        }
        // disable/kill/stop objects here
        sss.kill();
        robot.kill();
    }

    // misc functions here
    void updateEStop() {
        if(eStopButtonsReleased && ((gamepad1.dpad_down && gamepad1.back) || (gamepad2.dpad_down && gamepad2.back))) {
            eStopButtonsReleased = false;
            eStop = !eStop;
        } else if(!((gamepad1.dpad_down && gamepad1.back) || (gamepad2.dpad_down && gamepad2.back))) {
            eStopButtonsReleased = true;
        }
    }

    void controlDrive() {
        double drivePower = (slowMode)? leftStick.magnitude()/2.0 : leftStick.magnitude();
        double turnPower = (slowMode)? rightStick.x()/4.0 : rightStick.x();
        if(!eStop) robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    void controlStoneStackingSystem() {
        if(!eStop) {
            if (leftArmMode) {
                if (gamepad1.dpad_down) {
                    bothArmMode = true;
                    leftArmMode = false;
                    rightArmMode = false;
                } else if (gamepad1.dpad_right) {
                    bothArmMode = false;
                    leftArmMode = false;
                    rightArmMode = true;
                }

                if (gamepad1.a)
                    sss.setLeftArmPosition(StoneStackingSystemV1.LEFT_ARM_DEPLOY);
                else if (gamepad1.b)
                    sss.setLeftArmPosition(StoneStackingSystemV1.LEFT_ARM_RAISE);
                else if (!gamepad2.dpad_left && !gamepad2.dpad_right) // stop if player 2 not controlling
                    sss.setLeftArmPosition(StoneStackingSystemV1.LEFT_ARM_STOP);

                if (gamepad1.left_trigger > 0.1)
                    sss.extendLeftArm();
                else if (gamepad1.left_bumper)
                    sss.retractLeftArm();
                else if (Math.abs(gamepad2.left_stick_y) <= 0.1) // pause if player 2 not controlling
                    sss.pauseLeftArm();
            } else if (rightArmMode) {
                if (gamepad1.dpad_down) {
                    bothArmMode = true;
                    leftArmMode = false;
                    rightArmMode = false;
                } else if (gamepad1.dpad_left) {
                    bothArmMode = false;
                    leftArmMode = true;
                    rightArmMode = false;
                }

                if (gamepad1.a)
                    sss.setRightArmPosition(StoneStackingSystemV1.RIGHT_ARM_DEPLOY);
                else if (gamepad1.b)
                    sss.setRightArmPosition(StoneStackingSystemV1.RIGHT_ARM_RAISE);
                else if (!gamepad2.x && !gamepad2.b)  // stop if player 2 not controlling
                    sss.setRightArmPosition(StoneStackingSystemV1.RIGHT_ARM_STOP);

                if (gamepad1.left_trigger > 0.1)
                    sss.extendRightArm();
                else if (gamepad1.left_bumper)
                    sss.retractRightArm();
                else if (Math.abs(gamepad2.right_stick_y) <= 0.1) // pause if player 2 not controlling
                    sss.pauseRightArm();
            } else if (bothArmMode) {
                if (gamepad1.dpad_left) {
                    bothArmMode = false;
                    leftArmMode = true;
                    rightArmMode = false;
                } else if (gamepad1.dpad_right) {
                    bothArmMode = false;
                    leftArmMode = false;
                    rightArmMode = true;
                }

                if (gamepad1.a)
                    sss.deployArms();
                else if (gamepad1.b)
                    sss.liftArms();
                else if (!gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.x && !gamepad2.b) // stop if player 2 not controlling
                    sss.stopArms();

                if (gamepad1.left_trigger > 0.1) {
                    sss.extendRightArm();
                    sss.extendLeftArm();
                } else if (gamepad1.left_bumper) {
                    sss.retractRightArm();
                    sss.retractLeftArm();
                } else if (Math.abs(gamepad2.left_stick_y) <= 0.1 && Math.abs(gamepad2.right_stick_y) <= 0.1) { // pause if player 2 not controlling
                    sss.pauseRightArm();
                    sss.pauseLeftArm();
                }
            }

            if (gamepad1.right_trigger > 0.1) sss.liftStones();
            else if (gamepad1.right_bumper) sss.lowerStones();
            else if (!gamepad2.right_bumper && gamepad2.right_trigger <= 0.1) sss.pauseStoneLift(); // pause if player 2 not controlling
        }
        if(-gamepad2.left_stick_y > 0.1) sss.extendLeftArm();
        else if(-gamepad2.left_stick_y < -0.1) sss.retractLeftArm();
        else if (!gamepad1.left_bumper && gamepad1.left_trigger <= 0.1) sss.pauseLeftArm(); // pause if player 1 not controlling

        if(-gamepad2.right_stick_y > 0.1) sss.extendRightArm();
        else if(-gamepad2.right_stick_y < -0.1) sss.retractRightArm();
        else if (!gamepad1.left_bumper && gamepad1.left_trigger <= 0.1) sss.pauseRightArm();

        if(gamepad2.x) sss.setRightArmPosition(StoneStackingSystemV1.RIGHT_ARM_DEPLOY);
        else if(gamepad2.b) sss.setRightArmPosition(StoneStackingSystemV1.RIGHT_ARM_RAISE);
        else if(!gamepad1.a && !gamepad1.b) sss.setRightArmPosition(StoneStackingSystemV1.RIGHT_ARM_STOP); // stop if player 1 not controlling

        if(gamepad2.dpad_right) sss.setLeftArmPosition(StoneStackingSystemV1.LEFT_ARM_DEPLOY);
        else if(gamepad2.dpad_left) sss.setLeftArmPosition(StoneStackingSystemV1.LEFT_ARM_RAISE);
        else if(!gamepad1.a && !gamepad1.b) sss.setLeftArmPosition(StoneStackingSystemV1.LEFT_ARM_STOP);

        if (gamepad2.a)
            sss.grabStoneCenter();
        else if (gamepad2.y)
            sss.releaseStoneCenter();

        if(gamepad2.right_trigger > 0.1) sss.liftStones();
        else if(gamepad2.right_bumper) sss.lowerStones();
        else if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1) sss.pauseStoneLift(); // pause if player 1 not controlling
    }

    void stopActions() {
        sss.pauseStoneLift();
        sss.stopArms();
        robot.brake();
    }
}
