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

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
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
 *      start - slow mode
 *      a - shoot
 *      b - aim to right power shot
 *      x - aim to left power shot
 *      y - aim to middle power shot
 *      dpad up / down - raise / lower shooter angle
 *      dpad left / right - bring shooter all the way down / up
 *      right bumper - raise elevator
 *      left bumper - lower elevator
 *      right trigger - aim to top goal
 *
 * Player Two:
 *      b - intake direction
 *      a - intake power
 *      dpad up / down - raise / lower wobble grabber arm
 *      x - resets arm position for grabbing wobble goal
 *      y - grabbing or releasing wobble goal
 *      dpad up and down - raise and lower wobble grabber arm
 */

@TeleOp(name="Ultimate V1", group="Competition")
//@Disabled
public class UltimateV1 extends LinearOpMode {

    // TODO add speed values and angles when using the wobble grabber

    // create objects and locally global variables here
    UltimateNavigation robot;
    JoystickHandler leftStick, rightStick;
    GamepadController controllerOne, controllerTwo;

    RingIntakeSystemV1 intake;
    ShooterSystemV1 shooter;
    WobbleGrabberV1 grabber;

    boolean eStop = false, slowMode = false;

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
        intake = new RingIntakeSystemV1(hardwareMap);
        shooter = new ShooterSystemV1(hardwareMap, this);
        grabber = new WobbleGrabberV1(hardwareMap);

        // initialize joysticks
        leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);

        // initialize controllers
        controllerOne = new GamepadController(gamepad1);
        controllerTwo = new GamepadController(gamepad2);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // nothing goes between the above and below lines

        waitForStart();

        // puts the pinball servo on the outside
        shooter.pinballServo.setPosition(ShooterSystemV1.PINBALL_REST);

        // should only be used for a time keeper or other small things, avoid using this space when possible
        while (opModeIsActive()) {
            // main code goes here
//            telemetry.addData("Shooter angle: ", "" + shooter.aimServo.getPosition());
//            telemetry.update();

            updateEStop();
            if (!eStop) {

                // start button controls slow mode
                if (controllerOne.startPressed())
                    slowMode = !slowMode;

                updateEStop();

                controlDrive();

                updateEStop();

                if (!eStop) {
                    controllerOne.update();
                    controllerTwo.update();

                    playerOneFunctions(controllerOne);
                    playerTwoFunctions(controllerTwo);
                }
            }

            if (eStop)
                stopActions();

            // telemetry and logging data goes here
            telemetry.update();
        }

        // disable/kill/stop objects here
        robot.stopNavigation();
    }

    // misc functions here
    private void updateEStop() {
        if ((controllerOne.dpadDownHeld() && gamepad1.back) || (controllerTwo.dpadDownHeld() && gamepad2.back))
            eStop = !eStop;
    }

    private void controlDrive() {
        double drivePower = slowMode ? leftStick.magnitude() / 3 : leftStick.magnitude();
        double turnPower = slowMode ? rightStick.x() / 4 : rightStick.x();
        if (!eStop)
            robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    private void playerOneFunctions(GamepadController controller) {

        if (controller.aPressed())
            shooter.shoot();

        if (controllerOne.bPressed())
            shooter.keepElevatorAtTop();

        if (controllerOne.xPressed())
            shooter.setShooter(ShooterSystemV1.HIGHEST_POSITION);

        if (controller.dpadUpPressed())
            robot.turnToShoot(ConfigVariables.TOP_GOAL, this);  // set angle to aim at the top goal

        if (controller.dpadDownPressed())
            robot.turnToShoot(ConfigVariables.POWER_SHOT_MIDDLE, this); // set angle to center power shot

        if (controller.dpadLeftPressed())
            robot.turnToShoot(ConfigVariables.POWER_SHOT_LEFT, this); // set angle to left power shot

        if (controller.dpadRightPressed())
            robot.turnToShoot(ConfigVariables.POWER_SHOT_RIGHT, this); // set angle to right power shot

        if (controllerOne.leftBumperPressed())
            shooter.lowerElevator();
    }

    private void playerTwoFunctions(GamepadController controller) {

        if (controller.xPressed())
            shooter.toggleWheelPower();

        if (controller.yPressed())
            grabber.grabOrReleaseWobbleGoal();

        if (controller.dpadUpPressed())
            grabber.increaseAngle();

        if (controller.dpadDownPressed())
            grabber.decreaseAngle();

        if (controller.bPressed())
            intake.toggleIntake();

        if (controller.aPressed())
            intake.toggleOuttake();

        if (controller.startPressed())
            grabber.raiseToVertical();

        if (controller.rightBumperPressed())
            shooter.keepElevatorAtTop();

        if (controller.leftBumperPressed())
            shooter.lowerElevator();
    }

    private void stopActions() {
        robot.brake();
    }
}
