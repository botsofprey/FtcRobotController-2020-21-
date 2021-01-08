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
	
	public ServoHandler aimServo;
	public static final double HIGHEST_POSITION = 0;
	public static final double POWER_SHOT_POSITION = 0.45;
	public static final double LOWERED_POSITION = 1;
	final double ANGLE_INCREMENT = 0.05;
	
	// good
	public WheelMotor wheelMotor;
	private boolean wheelSpinning;
	private static final int SHOOTER_ON_SPEED = 3700; // rotations per minute
	private static final int HIGH_GOAL_SPEED = 0;
	private static final int POWER_SHOT_SPEED = 0;
	
	// good
	public ServoHandler elevatorServo;
	public static final int TOP = 2;
	public static final int MIDDLE = 1;
	public static final int BOTTOM = 0;
	public int elevatorPosition;
	public boolean stayAtTop;
	public volatile MagneticLimitSwitch elevatorTopSwitch;
	public volatile MagneticLimitSwitch elevatorBottomSwitch;
	
	// good
	public ServoHandler pinballServo;
	private double pinballAngle;
	public static final double PINBALL_TURNED = 1;
	public static final double PINBALL_REST = 0;
	
	private DistanceSensor ringDetector;
	private static final double RING_HEIGHT = 0.75;//inches
	private static final double DETECTOR_HEIGHT = 0;//the distance between the distance sensor and the hopper in inches //todo find this value
	
	private RevBlinkinLedDriver ringCountLEDs;
	private static final RevBlinkinLedDriver.BlinkinPattern[] RING_TABLE = {
			RevBlinkinLedDriver.BlinkinPattern.BLACK,
			RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY,
			RevBlinkinLedDriver.BlinkinPattern.GRAY,
			RevBlinkinLedDriver.BlinkinPattern.WHITE
	};
	
	public ShooterSystemV2Test(HardwareMap hardwareMap) {
		aimServo = new ServoHandler("aimServo", hardwareMap);
		aimServo.setDirection(Servo.Direction.FORWARD);
		
		wheelMotor = new WheelMotor("wheelMotor", "RPMDriver", hardwareMap);
		
		elevatorServo = new ServoHandler("elevatorServo", hardwareMap);
		elevatorServo.setDirection(Servo.Direction.FORWARD);
		elevatorTopSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorTopSwitch"));
		elevatorBottomSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorBottomSwitch"));
		
		pinballServo = new ServoHandler("pinballServo", hardwareMap);
		pinballServo.setDirection(Servo.Direction.FORWARD);
		
		ringDetector = hardwareMap.get(DistanceSensor.class, "ringDetector");
		
		wheelSpinning = false;
		elevatorPosition = BOTTOM;
		pinballAngle = PINBALL_REST;
		stayAtTop = false;
		
		ringCountLEDs = hardwareMap.get(RevBlinkinLedDriver.class, "ringCountDriver");
	}
	
	public void turnOnShooterWheel() {
		wheelMotor.setRPM(SHOOTER_ON_SPEED);
	}
	
	public void turnOffShooterWheel() {
		wheelMotor.setRPM(0);
	}
	
	public void toggleShooterWheel() {
		if (wheelMotor.targetRPM == 0)
			wheelMotor.setRPM(SHOOTER_ON_SPEED);
		else
			wheelMotor.setRPM(0);
	}
	
	// moves the pinball servo
	public void shoot() {
		if (pinballAngle == PINBALL_REST) {
			pinballAngle = PINBALL_TURNED;
		}
		else {
			pinballAngle = PINBALL_REST;
		}
		
		pinballServo.setPosition(pinballAngle);
	}
	
	public void setShooter(double angle) { aimServo.setPosition(angle); }
	
	public void aimAtPowerShot() {
		wheelMotor.setRPM(POWER_SHOT_SPEED);
	}
	
	public void aimAtHighGoal() {
		wheelMotor.setRPM(HIGH_GOAL_SPEED);
	}
	
	public void raiseElevator() {
		if (elevatorPosition != TOP)
			elevatorServo.setPosition(0);
	}
	
	public void lowerElevator() {
		stayAtTop = false;
		if (elevatorPosition != BOTTOM)
			elevatorServo.setPosition(1);
	}
	
	public void keepElevatorAtTop() {
		stayAtTop = true;
		raiseElevator();
	}
	
	public void stopElevator() { elevatorServo.setPosition(0.5); }
	
	public void update() {
		if (elevatorTopSwitch.isActivated() && elevatorPosition != TOP) {
			elevatorPosition = TOP;
			elevatorServo.setPosition(0.5);
		} else if (elevatorBottomSwitch.isActivated() && !stayAtTop) { //watch out for the zero case because then the robot will think its at the bottom when its at the top
			elevatorPosition = BOTTOM;
			elevatorServo.setPosition(0.5);
		} else if (!elevatorTopSwitch.isActivated() && stayAtTop)
			elevatorServo.setPosition(0);
		
		if (!elevatorTopSwitch.isActivated() && !elevatorBottomSwitch.isActivated())
			elevatorPosition = MIDDLE;
		
		wheelMotor.updateShooterRPM();
		
		
	}
	
	public double calculateRingVelocity(double xDistance, double yDistance) {
		double temp0 = yDistance - xDistance * Math.tan(ConfigVariables.SHOOTER_ANGLE);
		if (temp0 < 0)
			return 0;
		double temp1 = Math.sqrt(-4.9 * xDistance * temp0);
		return Math.cos(ConfigVariables.SHOOTER_ANGLE) / temp1;
	}
	
	private int getNumRingsInHopper() {
		double distance = ringDetector.getDistance(DistanceUnit.INCH);
		distance -= DETECTOR_HEIGHT;
		distance *= -1;
		distance /= RING_HEIGHT;
		for (int i = 0; i < 2; i++)
			if (Math.abs(distance - i) < RING_HEIGHT / 2)
				return i;
		return 3;
	}
	
	private void signalRingCount(int numRings) {
		ringCountLEDs.setPattern(RING_TABLE[numRings]);
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
	public void kill() {}
}
