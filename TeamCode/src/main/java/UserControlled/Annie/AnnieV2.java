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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Actions.Annie.MiscellaneousActionsV2;
import Actions.Annie.StoneStackingSystemV3;
import Autonomous.Location;
import DriveEngine.HolonomicDriveSystemTesting;
import SensorHandlers.SensorPackage;
import SensorHandlers.UltrasonicIRSensor;
import UserControlled.JoystickHandler;

@TeleOp(name="Annie V2", group="Competition")
//@Disabled
public class AnnieV2 extends LinearOpMode {
    // create objects and locally global variables here
    HolonomicDriveSystemTesting robot;
    StoneStackingSystemV3 sss;
    MiscellaneousActionsV2 otherActions;
    SensorPackage sensors;
    JoystickHandler leftStick, rightStick;
    boolean eStop = false, slowMode = false, superSlowMode = false, tapeStopped = true, liftLowered = true, liftingToPos = false, capstoneDeploy = false;
    boolean startReleased = true, eStopButtonsReleased = true, limitSwitchReleased = false,
            rightTrigger1Released = true, rightBumper1Released = true,
            p2DpadUpReleased = true;
    int stonePosition = 0, capstoneLocation = 0;
    long timeDeploy = 0;
    @Override
    public void runOpMode() {
        // initialize objects and variables here
        // also create and initialize function local variables here
        robot = new HolonomicDriveSystemTesting(hardwareMap, new Location(0, 0), 0, "RobotConfig/AnnieV1.json");

        sss = new StoneStackingSystemV3(hardwareMap);
        otherActions = new MiscellaneousActionsV2(hardwareMap);

        sensors = new SensorPackage(/*new MagneticLimitSwitch(hardwareMap.get(DigitalChannel.class, "liftReset"), "liftReset"),*/
                new UltrasonicIRSensor(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front"), "front"));

        leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
        rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);

        // add any other useful telemetry data or logging data here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // nothing goes between the above and below lines
        waitForStart();
        // should only be used for a time keeper or other small things, avoid using this space when possible
        long startTime = System.currentTimeMillis();
//        otherActions.retractTape();
        while (opModeIsActive()) {
            // main code goes here
//            if(System.currentTimeMillis() - startTime > 3500 && !tapeStopped) {
//                otherActions.pauseTape();
//                tapeStopped = true;
//            }

//            telemetry.addData("M0:", sss.getLiftPositionTicks(0));
//            telemetry.addData("M1:", sss.getLiftPositionTicks(1));
//            telemetry.update();

            updateEStop();
            if(!eStop) {
                updateEStop();
                controlDrive();

                updateEStop();
                controlStoneStackingSystem();
                updateCapstone();
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
        if(gamepad1.dpad_down) {
            while (opModeIsActive() && sensors.getSensor(UltrasonicIRSensor.class, "front").getDistance() < 3.25) {
                robot.driveOnHeadingPID(180, 0.2, robot.orientation.getOrientation());
            }
            robot.brake();
        }

        if (startReleased && gamepad1.start) {
            startReleased = false;
            superSlowMode = !superSlowMode;
        } else if (!gamepad1.start) {
            startReleased = true;
        }

        slowMode = (gamepad1.right_trigger > 0.1);

        double drivePower = 0, turnPower = 0;
        if(superSlowMode) {
            drivePower = leftStick.magnitude()/4.0;
            turnPower = rightStick.x()/5.0;
        } else if(slowMode) {
            drivePower = leftStick.magnitude()/3.0;
            turnPower = rightStick.x()/4.0;
        } else {
            drivePower = leftStick.magnitude();
            turnPower = rightStick.x();
        }

        if(!eStop) robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
    }

    void controlStoneStackingSystem() {
        if(!eStop) {

            //PLAYER 1

            if(gamepad1.a) otherActions.grabFoundation();
            else if(gamepad1.b) otherActions.releaseFoundation();

            if(gamepad1.left_trigger > 0.1) deployCapstone();
            else if(gamepad1.left_bumper) releaseCapstone();


            // PLAYER 2

            if(gamepad2.right_trigger > 0.1) {
                sss.liftStones();
                liftingToPos = false;
            }
            else if(gamepad2.right_bumper/* && !sensors.getSensor(LimitSwitch.class, "liftReset").isPressed()*/) {
                sss.lowerStones();
                liftingToPos = false;
            }
            else if(!liftingToPos) sss.pauseStoneLift();
//            telemetry.addData("Lift: ", sss.getLiftPositionTicks());
            telemetry.update();

            if(gamepad2.left_trigger > 0.1) otherActions.spitTape();
            else if(gamepad2.left_bumper) otherActions.retractTape();
            else if(tapeStopped) otherActions.pauseTape();

            if (gamepad2.a)
                sss.grabStoneCenter();
            else if (gamepad2.y)
                sss.releaseStoneCenter();
            // check if limit switch is pressed and reset the lift encoder
//            if (sensors.getSensor(LimitSwitch.class, "liftReset").isPressed() && limitSwitchReleased) {
//                limitSwitchReleased = false;
//                sss.resetLiftEncoder();
//            } else if (!sensors.getSensor(LimitSwitch.class, "liftReset").isPressed() && !limitSwitchReleased) {
//                limitSwitchReleased = true;
//            }
        }
    }

    void deployCapstone(){
        capstoneDeploy = true;
        timeDeploy = System.currentTimeMillis();
    }
    void releaseCapstone(){
        capstoneLocation = 0;
        capstoneDeploy = false;
        sss.releaseCapstone();
    }
    void updateCapstone(){
        if(System.currentTimeMillis() - timeDeploy > 13) {
            timeDeploy = System.currentTimeMillis();
            if(capstoneDeploy) capstoneLocation++;
            if(capstoneLocation > 180){
                capstoneLocation = 180;
                capstoneDeploy = false;
            }
//            if(capstoneLocation < 0) capstoneLocation = 0;
        }
        sss.setCapstoneDegree(capstoneLocation);
    }

    void stopActions() {
        sss.pauseStoneLift();
        otherActions.pauseTape();
        robot.brake();
    }
}
