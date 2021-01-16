package Actions.Ultimate;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import Autonomous.ConfigVariables;
import MotorControllers.MotorController;
import SensorHandlers.MagneticLimitSwitch;

/**
 * Author: Jordan Martin
 * Date: 12/19/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV2Test implements ActionHandler {
	// good
	public WheelMotor wheelMotor;
	private boolean wheelSpinning;
	
	// TODO TEST FOR ACTUAL RPMS
	private static final int SHOOTER_ON_SPEED = 3700; // rotations per minute
	private static final int SHOOTER_OFF_SPEED = 0;
	private static final int HIGH_GOAL_SPEED = 0;
	private static final int POWER_SHOT_SPEED = 0;
	
	
	// good
	public ServoHandler indexServo;
	private double indexAngle;
	public static final double INDEX_LEFT = -1;
	public static final double INDEX_RIGHT = 1;
	
	
	private RevBlinkinLedDriver ringCount;
	private DistanceSensor ringDetector;
	private static final double RING_DETECTOR_HEIGHT = 0;//todo find this value
	private static final RevBlinkinLedDriver.BlinkinPattern[] COLORS = {
			RevBlinkinLedDriver.BlinkinPattern.BLACK,
			RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY,
			RevBlinkinLedDriver.BlinkinPattern.GRAY,
			RevBlinkinLedDriver.BlinkinPattern.WHITE
	};
	
	public ShooterSystemV2Test(HardwareMap hardwareMap) {
		wheelMotor = new WheelMotor("wheelMotor", hardwareMap);
		
		indexServo = new ServoHandler("indexServo", hardwareMap);
		indexServo.setDirection(Servo.Direction.FORWARD);
		
		wheelSpinning = false;
		indexAngle = INDEX_LEFT;
		
		ringDetector = hardwareMap.get(DistanceSensor.class, "ringDetector");
	}
	
	public void turnOnShooterWheel() {
		wheelMotor.setRPM(SHOOTER_ON_SPEED);
	}
	
	public void turnOffShooterWheel() {
		wheelMotor.setRPM(SHOOTER_OFF_SPEED); // use brake instead?
	}
	
	public void toggleShooterWheel() {
		if (wheelMotor.targetRPM == 0)
			wheelMotor.setRPM(SHOOTER_ON_SPEED);
		else
			wheelMotor.setRPM(SHOOTER_OFF_SPEED);
	}
	
	// moves the pinball servo
	public void setIndexLeft(){
		indexServo.setPosition(INDEX_LEFT);
	}
	public void setIndexRight(){
		indexServo.setPosition(INDEX_RIGHT);
	}
	
	public void setPowerShotSpeed() {
		wheelMotor.setRPM(POWER_SHOT_SPEED);
	}
	
	public void setHighGoalSpeed() {
		wheelMotor.setRPM(HIGH_GOAL_SPEED);
	}
	
	public void update() {
		wheelMotor.updateShooterRPM();
		
//		updateRingLEDs();
	}
	
	public double calculateRingVelocity(double xDistance, double yDistance) {
		double temp0 = yDistance - xDistance * Math.tan(ConfigVariables.SHOOTER_ANGLE);
		if (temp0 < 0)
			return 0;
		double temp1 = Math.sqrt(-4.9 * xDistance * temp0);
		return Math.cos(ConfigVariables.SHOOTER_ANGLE) / temp1;
	}
	
//	private void updateRingLEDs() {
//		double stackHeight = RING_DETECTOR_HEIGHT - ringDetector.getDistance(DistanceUnit.INCH);
//		int numRings = (int)Math.round(stackHeight / .75);
//		ringCount.setPattern(COLORS[numRings]);
//	}
	
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
	public void kill() {}
}
