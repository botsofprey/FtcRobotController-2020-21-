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

package UserControlled.Ultimate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.Ultimate.RingIntakeSystemV2Test;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2Test;
import Autonomous.ConfigVariables;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigation;
import UserControlled.GamepadController;
import UserControlled.JoystickHandler;

/**
 * Author: Software Team 2020-2021
 *
 * Controls the Ultimate Goal Robot
 *
 * -------------- TLDR ---------------
 * Player One:
 *      joysticks - drive base
 *      start - N/A
 *      a - N/A
 *      b - N/A
 *      x - N/A
 *      y - N/A
 *      dpad up/down/left/right - auto power shots
 *      right trigger - shoot
 *      left trigger - slow mode
 *
 *      right bumper - increase rpm by 100
 *      left bumper - decrease rpm by 100
 *
 * Player Two:
 *      joysticks - N/A
 *      a - toggle intake
 *      b - toggle outake
 *      x - toggle shooter
 *      y - toggle wobble grabber
 *      dpad up/down/left/right - wobble grabber positions
 *      right trigger - drop intake
 *      left trigger - N/A
 */

@TeleOp(name="Ultimate V2", group="Competition")
//@Disabled
public class UltimateV2Better extends LinearOpMode {

    // TODO add speed values and angles when using the wobble grabber

    // create objects and locally global variables here
    UltimateNavigation robot;
    JoystickHandler leftStick, rightStick;

    RingIntakeSystemV2Test intake;
    ShooterSystemV2Test shooter;
    WobbleGrabberV2Test grabber;
    
    GamepadController controllerOne, controllerTwo;

    boolean eStop = false, slowMode = false, intakeOn = false, outakeOn = false, shooterOn = false, wobbleGrabbed = false, a2Pressed = false, b2Pressed = false, x2Pressed = false, y2Pressed = false;

    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here

        // initialize robot
        // TODO get starting angle
        try {
            robot = new UltimateNavigation(hardwareMap, new Location(0, 0, 0), "RobotConfig/UltimateV1.json");
        } catch (Exception e) {
            telemetry.addData("Robot Error", e.toString());
            telemetry.update();
        }

        // initialize systems
        intake = new RingIntakeSystemV2Test(hardwareMap);
        shooter = new ShooterSystemV2Test(hardwareMap);
        grabber = new WobbleGrabberV2Test(hardwareMap);

        // initialize joysticks
        leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);
    
        controllerOne = new GamepadController(gamepad1);//todo use these for taking inputs; they improve code readability and make it simpler
        controllerTwo = new GamepadController(gamepad2);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // nothing goes between the above and below lines

        waitForStart();

        // puts the pinball servo on the outside
        shooter.pinballServo.setPosition(ShooterSystemV1.PINBALL_REST);
        shooter.update();

        // should only be used for a time keeper or other small things, avoid using this space when possible
        while (opModeIsActive()) {
            // main code goes here

            updateEStop();
            if (!eStop) {
                updateEStop();
                controlDrive();

                updateEStop();
                if (!eStop) {
                    playerOneFunctions();
                    playerTwoFunctions();
                }
            }

            if (eStop)
                stopActions();

            // telemetry and logging data goes here
            telemetry.update();
        }

        // disable/kill/stop objects here
        intake.kill();
        shooter.kill();
        grabber.kill();
        robot.stopNavigation();

    }

    // misc functions here
    private void updateEStop() {
        if ((gamepad1.dpad_down && gamepad1.start) || (gamepad2.dpad_down && gamepad2.start))
            eStop = !eStop;
    }

    private void controlDrive() {
        if(gamepad1.left_trigger > 0.1) slowMode = true;
        else slowMode = false;
        double drivePower = slowMode ? leftStick.magnitude() / 3 : leftStick.magnitude();
        double turnPower = slowMode ? rightStick.x() / 4 : rightStick.x();
        if (!eStop)
            robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    private void playerOneFunctions() {
        if(gamepad1.dpad_up) powerShots();
        else if(gamepad1.dpad_left) powerShotLeft();
        else if(gamepad1.dpad_down) powerShotCenter();
        else if(gamepad1.dpad_right) powerShotRight();

        if(gamepad1.right_trigger > 0.1) shooter.shoot();

        // TODO: update this to be actually correct, need to determine which wall to be against and what the x and y values would be
        if(gamepad1.x) {
            robot.setLocation(new Location(0, robot.getRobotLocation().getY()));
        } else if(gamepad1.y) {
            robot.setLocation(new Location(robot.getRobotLocation().getX(), 0));
        }
        
        if (controllerOne.leftBumperPressed())
            shooter.wheelMotor.setRPM((int)shooter.wheelMotor.targetRPM - 100);
        
        if (controllerOne.rightBumperPressed())
            shooter.wheelMotor.setRPM((int)shooter.wheelMotor.targetRPM + 100);
    }

    private void playerTwoFunctions() {
        if (controllerTwo.aPressed())
            intake.updateState(0);
        
        else if (controllerTwo.bPressed())
            intake.updateState(1);

        if(gamepad2.x && !x2Pressed) {
            x2Pressed = true;
            shooterOn = !shooterOn;
        } else if(!gamepad2.x) {
            x2Pressed = false;
        }

        if(gamepad2.y && !y2Pressed) {
            y2Pressed = true;
            wobbleGrabbed = !wobbleGrabbed;
        } else if(!gamepad2.y) {
            y2Pressed = false;
        }

        if(gamepad2.dpad_up) {
            grabber.setArmAngle(WobbleGrabberV2Test.GAINS_ANGLE);
        } else if(gamepad2.dpad_right) {
            grabber.setArmAngle(WobbleGrabberV2Test.LIFT_ANGLE);
        } else if(gamepad2.dpad_down) {
            grabber.setArmAngle(WobbleGrabberV2Test.GRAB_AND_DROP_ANGLE);
        } else if(gamepad2.dpad_left) {
            grabber.setArmAngle(WobbleGrabberV2Test.INIT_ANGLE);
        }
//        if(gamepad2.right_trigger > 0.1) intake.drop();
        if(wobbleGrabbed) grabber.grabWobble();
        else if(!wobbleGrabbed) grabber.releaseWobble();
        if(shooterOn) shooter.turnOnShooterWheel();
        else if(!shooterOn) shooter.turnOffShooterWheel();
        shooter.update();
    }

    private void powerShots() {
        powerShotLeft();
        powerShotCenter();
        powerShotRight();
    }

    // TODO: Modify the functions below to actually go to the correct positions and score power shots

    private void powerShotLeft() {
        shooter.turnOnShooterWheel();
        robot.driveToLocation(ConfigVariables.POWER_SHOT_LEFT_ON_LINE, 25, this);
        shooter.shoot();
        sleep(10);
        shooter.shoot();
    }

    private void powerShotCenter() {
        shooter.turnOnShooterWheel();
        robot.driveToLocation(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
        shooter.shoot();
        sleep(10);
        shooter.shoot();
    }

    private void powerShotRight() {
        shooter.turnOnShooterWheel();
        robot.driveToLocation(ConfigVariables.POWER_SHOT_RIGHT_ON_LINE, 25, this);
        shooter.shoot();
        sleep(10);
        shooter.shoot();
    }

    private void stopActions() {
        robot.brake();
        intake.intakeOff();
        grabber.pause();
        shooter.turnOffShooterWheel();
    }
}
