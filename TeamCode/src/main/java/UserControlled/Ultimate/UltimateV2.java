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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import Actions.Ultimate.RingIntakeSystemV2;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigationSimple;
import UserControlled.GamepadController;
import UserControlled.JoystickHandler;

import static Autonomous.ConfigVariables.HIGH_GOAL_HEADING;
import static Autonomous.ConfigVariables.LEFT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.MIDDLE_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.RIGHT_POWER_SHOT_HEADING;
import static DriveEngine.Ultimate.UltimateNavigationSimple.RIGHT_SENSOR;

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
 *      x - reset robot x position against left wall
 *      y - reset robot y position against back wall
 *      dpad up/down/left/right - auto power shots
 *      right trigger - shoot
 *      left trigger - slow mode
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
 *      right trigger - toggle intake servo
 *      left trigger - N/A
 */

@TeleOp(name="Ultimate V2", group="Competition")
//@Disabled
public class UltimateV2 extends LinearOpMode {

	// create objects and locally global variables here
	UltimateNavigationSimple robot;
	JoystickHandler leftStick, rightStick;
	
	RingIntakeSystemV2 intake;
	ShooterSystemV2Test shooter;
	WobbleGrabberV2 grabber;
	
	GamepadController controllerOne, controllerTwo;

	protected static final double MAX_SPEED = 50;
	protected static final double MED_SPEED = 25;
	protected static final double LOW_SPEED = 15;
	protected static final double MIN_SPEED = 5;
	protected static final long SLEEP_TIME = 550;

	long indexTimer = 0;
	
	boolean eStop = false, slowMode = false, intakeOn = false, outakeOn = false, y2Pressed = false, x2Pressed = false, toggleShooterWheel = false, toggleWobbleGrabbed = false,
			rt1Pressed = false, rightTriggerPressed = false, toggleIndex = false, toggleIntakeServo = false, rt2Pressed = false, a2Pressed = false, b2Pressed = false,
			dpadD2pressed = false, dpadU2pressed = false, toggleDecrement = false, x1Pressed = false, usingDpad = false, extraSlowMode = false, start1Pressed = false, shouldCheckArmLimit = false,
			indexed = true;
	
	@Override
	public void runOpMode() {
		// initialize objects and variables here
		// also create and initialize function local variables here
		
		// initialize robot
		// TODO get starting angle
		try {
			robot = new UltimateNavigationSimple(hardwareMap, new Location(0, 0, 0), "RobotConfig/UltimateV2.json");
			Log.d("Robot: ", robot.toString());
		} catch (Exception e) {
			telemetry.addData("Robot Error", e.toString());
			telemetry.update();
		}
		
		// initialize systems
		intake = new RingIntakeSystemV2(hardwareMap);
		shooter = new ShooterSystemV2Test(hardwareMap);
		grabber = new WobbleGrabberV2(hardwareMap);

		// initialize joysticks
		leftStick = new JoystickHandler(gamepad1, JoystickHandler.LEFT_JOYSTICK);
		rightStick = new JoystickHandler(gamepad1, JoystickHandler.RIGHT_JOYSTICK);
		controllerOne = new GamepadController(gamepad1);
		controllerTwo = new GamepadController(gamepad2);
		
		// add any other useful telemetry data or logging data here
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		
		// nothing goes between the above and below lines
		
		waitForStart();

		
		// should only be used for a time keeper or other small things, avoid using this space when possible
		while (opModeIsActive()) {
			// main code goes here
			
			updateEStop();
			if (!eStop) {
				updateEStop();
				controllerOne.update();
				controllerTwo.update();
				
				controlDrive();
				
				updateEStop();
				controllerOne.update();
				controllerTwo.update();
				if (!eStop) {
					playerOneFunctions(controllerOne);
					playerTwoFunctions(controllerTwo);
				}
				telemetry.addData("Wheel Power", shooter.shooterMotor.getMotorPower());
				telemetry.addData("Wheel Speed (ticks/sec)", shooter.shooterMotor.getCurrentTicksPerSecond());
				telemetry.addData("Shooter RPM", shooter.getRPM());
				Log.d("ShooterRPM", shooter.getRPM() + "");
				Log.d("ShooterVelocity", ((DcMotorEx) (shooter.shooterMotor.getMotor())).getVelocity() + "");
				telemetry.addData("Robot Heading", robot.orientation.getOrientation());
				telemetry.addData("Wobble Angle", grabber.arm.getDegree());
//				telemetry.addData("Right Dist", robot.distanceSensors[RIGHT_SENSOR].getDistance());
				telemetry.update();

				updateEStop();
				if (!eStop)
					updateMiscFunctions();
			}
			
			// telemetry and logging data goes here
			telemetry.update();
		}
		
		// disable/kill/stop objects here
		stopActions();
		intake.kill();
		shooter.kill();
		grabber.kill();
		robot.kill();
	}
	
	// misc functions here
	private void updateEStop() {
		if ((controllerOne.dpadDownHeld && controllerOne.startHeld) || (controllerTwo.dpadDownHeld && controllerTwo.startHeld))
			eStop = !eStop;
	}
	
	private void controlDrive() {
		slowMode = controllerOne.leftTriggerHeld;

		if(gamepad1.start && !start1Pressed) {
			start1Pressed = true;
			extraSlowMode = !extraSlowMode;
		} else if(start1Pressed && !gamepad1.start) {
			start1Pressed = false;
		}

//		double drivePower = slowMode ? leftStick.magnitude() / 1.5 : leftStick.magnitude();
//		double turnPower = slowMode ? rightStick.x() / 2.0 : rightStick.x() / 1.3;

		double drivePower = leftStick.magnitude();
		double turnPower = rightStick.x();
		if(slowMode) {
			drivePower /= 1.5;
			turnPower /= 2.5;
		} else if(extraSlowMode) {
			drivePower /= 2.0;
			turnPower /= 3.0;
		} else turnPower /= 1.3;
		if (!eStop)
			robot.driveOnHeadingWithTurning(leftStick.angle(), drivePower, turnPower);
	}
	
	private void playerOneFunctions(GamepadController controller) {
		if(gamepad1.dpad_up) {
			shooter.setPowerShotRPM();
		}
		else if(gamepad1.dpad_left) {
//			robot.driveToXY(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
			shooter.setPowerShotRPM();
		}
		else if(gamepad1.dpad_down) {
//			robot.driveToXY(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
			powerShotCenter();
		}
		else if(gamepad1.dpad_right) {
//			robot.driveToXY(ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE, 25, this);
			powerShotRight();
		}


		// Indexer toggle
		if(indexed && gamepad1.right_trigger > 0.1 && !rt1Pressed){
			rt1Pressed = true;
			indexed = false;
			indexTimer = System.currentTimeMillis();
		}
		else if(!(gamepad1.right_trigger > 0.1)){
			rt1Pressed = false;
		}
		if(!indexed){
			indexShooter();
		}

		// Shoot three rings
		if(gamepad1.x && !x1Pressed) {
			x1Pressed = true;
//			shootThreeRings();
			highGoalShot();
		}
		else if(!gamepad1.x){
			x1Pressed = false;
		}
	}

	private void highGoalShot() {
		shooter.setShooterMotorRPM(ShooterSystemV2Test.HIGH_GOAL_RPM);
		robot.turnToHeadingEnhanced(HIGH_GOAL_HEADING, 0.25,this);
		indexed = false;
		while (opModeIsActive() && !indexed) indexShooter();
	}
	
	private void playerTwoFunctions(GamepadController controller) {
		if(gamepad2.a && !a2Pressed) {
			a2Pressed = true;
			intakeOn = !intakeOn;
			outakeOn = false;
		} else if(!gamepad2.a) {
			a2Pressed = false;
		}

		if(gamepad2.b && !b2Pressed) {
			b2Pressed = true;
			outakeOn = !outakeOn;
			intakeOn = false;
		} else if(!gamepad2.b) {
			b2Pressed = false;
		}

		if(intakeOn) intake.intake();
		else if(outakeOn) intake.spit();
		else intake.pauseIntake();

		// Shooter wheel toggle
		if (gamepad2.x && !x2Pressed) {
			x2Pressed = true;
			toggleShooterWheel = !toggleShooterWheel;
		} else if (!gamepad2.x) {
			x2Pressed = false;
		}
		if (toggleShooterWheel) {
			shooter.spinUp();
		} else {
			shooter.pauseShooter();
		}

		// Wobble grab toggle
		if (gamepad2.y && !y2Pressed) {
			y2Pressed = true;
			toggleWobbleGrabbed = !toggleWobbleGrabbed;
		} else if (!gamepad2.y) {
			y2Pressed = false;
		}
		if (toggleWobbleGrabbed) {
			grabber.setClawGrabAngle();
		} else {
			grabber.releaseWobble();
		}

		if (gamepad2.dpad_up) {
			usingDpad = true;
			shouldCheckArmLimit = false;
			grabber.setWallAngle();
		} else if (gamepad2.dpad_right) {
			usingDpad = true;
			shouldCheckArmLimit = false;
			grabber.setLiftAngle();
		} else if (gamepad2.dpad_down) {
			usingDpad = true;
			shouldCheckArmLimit = false;
			grabber.setGrabAndDropAngle();
		} else if (gamepad2.dpad_left) {
			usingDpad = true;
			shouldCheckArmLimit = true;
			grabber.setInitAngle();
		}

		if(gamepad2.right_trigger > 0.1) {
			usingDpad = false;
			shouldCheckArmLimit = false;
			grabber.setArmPower(0.55);
		} else if(gamepad2.left_trigger > 0.1) {
			usingDpad = false;
			shouldCheckArmLimit = true;
			grabber.setArmPower(-0.55);
		} else if(!usingDpad) {
			grabber.holdArm();
		}

//		if(grabber.armLimit.isActivated()) {
//			grabber.resetArm(this);
//		}
		if(gamepad2.start) {
			grabber.arm.brake();
			grabber.resetArm(this);
		}

		// Intake servo toggle
		if (gamepad2.right_trigger > 0.1 && !rt2Pressed) {
			rt2Pressed = true;
			toggleIntakeServo = !toggleIntakeServo;
		} else if (!(gamepad2.right_trigger > 0.1)) {
			rt2Pressed = false;
		}
		if (toggleIntakeServo) {
			intake.intakeServoOut();
		} else {
			intake.intakeServoIn();
		}

		if (gamepad2.right_bumper) {
			shooter.setHighGoalRPM();
		}

		if(gamepad2.left_bumper) {
			shooter.setRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 150);
		}
	}
	
	private void powerShots() {
		powerShotRight();
		powerShotCenter();
		powerShotLeft();
	}
	
	// TODO: Modify the functions below to actually go to the correct positions and score power shots
	
	private void powerShotLeft() {
		shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 150);
		robot.turnToHeadingEnhanced(LEFT_POWER_SHOT_HEADING, 0.25, this);
		indexed = false;
		while (opModeIsActive() && !indexed) indexShooter();
	}
	
	private void powerShotCenter() {
		shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 150);
		robot.turnToHeadingEnhanced(MIDDLE_POWER_SHOT_HEADING, 0.25, this);
		indexed = false;
		while (opModeIsActive() && !indexed) indexShooter();
	}
	
	private void powerShotRight() {
		shooter.setShooterMotorRPM(ShooterSystemV2Test.POWER_SHOT_RPM + 150);
		robot.turnToHeadingEnhanced(RIGHT_POWER_SHOT_HEADING, 0.25, this);
		indexed = false;
		while (opModeIsActive() && !indexed) indexShooter();
	}

	protected void indexShooter(){
		indexed = false;
		shooter.setIndexLeft();
		if(System.currentTimeMillis() - indexTimer >= SLEEP_TIME) {
			shooter.setIndexRight();
			indexed = true;
		}
	}
	
	private void stopActions() {
		robot.brake();
		intake.intakeOff();
		grabber.pause();
//		shooter.turnOffShooterWheel();
		shooter.pauseShooter();
	}
	
	private void updateMiscFunctions() {
//		shooter.update();
		if(shouldCheckArmLimit) grabber.armSensorCheck(this);
	}
}
