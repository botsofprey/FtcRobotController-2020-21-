package Actions.Ultimate;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import Autonomous.ConfigVariables;
import MotorControllers.MotorController;
import MotorControllers.MotorControllerTest;
import SensorHandlers.LIDARSensor;

/**
 * Author: Jordan Martin
 * Date: 12/19/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV2Test implements ActionHandler {

	private static final double DISTANCE_TO_RINGS = 0; // TODO find this value
	private static final double RING_WIDTH = 0.75;
	public MotorControllerTest shooterMotor;

	// TODO fix PID controller in rpm class
	private static final int SHOOTER_ON_SPEED = 5000; // rotations per minute
	private static final int SHOOTER_OFF_SPEED = 0;
	private static final int HIGH_GOAL_SPEED = 0;
	private static final int POWER_SHOT_SPEED = 0;


	// TODO USE THESE POWERS
	private static final double SHOOTER_OFF_POWER = 0;
	public static final double HIGH_GOAL_POWER = 0.655; // .65
	public static final double RIGHT_POWER_SHOT_POWER = 0.615;
	public static final double MIDDLE_POWER_SHOT_POWER = 0.595;
	public static final double LEFT_POWER_SHOT_POWER = 0.58;
	public static final double POWER_SHOT_POWER = 0.615;

	public static final double HIGH_GOAL_RPM = 3915;
	public static final double RIGHT_POWER_SHOT_RPM = 3000;
	public static final double MIDDLE_POWER_SHOT_RPM = 2800;
	public static final double LEFT_POWER_SHOT_RPM = 2800;
	public static final double POWER_SHOT_RPM = 3500;
	public static double rpm = HIGH_GOAL_RPM;

	private double power = HIGH_GOAL_POWER;

	// good
	public ServoHandler indexServo;
	private double indexAngle;
	public static final double INDEX_LEFT = -1;
	public static final double INDEX_RIGHT = 1;

	public LIDARSensor hopperSensor;



	public ShooterSystemV2Test(HardwareMap hardwareMap) {
		try {
			shooterMotor = new MotorControllerTest("wheelMotor", "ActionConfig/ShooterMotor.json", hardwareMap);
			shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		} catch (IOException e) {
			e.printStackTrace();
		}

		indexServo = new ServoHandler("indexServo", hardwareMap);
		indexServo.setDirection(Servo.Direction.FORWARD);

		indexAngle = INDEX_LEFT;


	}

	public void spinUp() {
		shooterMotor.setRPM(rpm);
		Log.d("Shooter RPM set", rpm + "");
	}

	public void setRPM(double rpm) {
		this.rpm = rpm;
		spinUp();
		Log.d("Shooter", "set rpm");
	}

	public double getRPM() { return shooterMotor.getCurrentRPM(); }

	public PIDFCoefficients getPID() {
		return shooterMotor.getRPMPID();
	}

	public void pauseShooter() {
		shooterMotor.brake();
	}

	// moves the index servo
	public void setIndexLeft(){
		indexServo.setPosition(INDEX_LEFT);
		indexAngle = INDEX_LEFT;
	}

	public void setIndexRight(){
		indexServo.setPosition(INDEX_RIGHT);
		indexAngle = INDEX_RIGHT;
	}

	public void setHighGoalRPM() {
		rpm = HIGH_GOAL_RPM;
		spinUp();
	}

	public void setShooterMotorRPM(double rpm) {
		this.rpm = rpm;
		spinUp();
	}

	public void setPowerShotRPM() {
		rpm = POWER_SHOT_RPM;
		spinUp();
	}

	public int getRingsInHopper() {
		double distance = hopperSensor.getDistance();
		distance -= DISTANCE_TO_RINGS;
		distance /= RING_WIDTH;
		return 3 - (int)Math.round(distance);
	}

	@Override
	public boolean doAction(String action, long maxTimeAllowed) {
		return false;
	}
	
	@Override
	public boolean stopAction(String action) {
		return false;
	}
	
	@Override
	public boolean startDoingAction(String action) {
		return false;
	}
	
	@Override
	public void kill() {
		shooterMotor.killMotorController();
	}
}
