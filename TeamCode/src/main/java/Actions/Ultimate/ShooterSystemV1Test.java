package Actions.Ultimate;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.HardwareWrappers.ServoHandler;
import Autonomous.ConfigVariables;
import MotorControllers.MotorController;
import SensorHandlers.MagneticLimitSwitch;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for shooting rings
 */
public class ShooterSystemV1Test {

    public ServoHandler aimServo;
    public static final double HIGHEST_POSITION = 0;
    public static final double POWER_SHOT_POSITION = 0.45;
    public static final double LOWERED_POSITION = 1;
    final double ANGLE_INCREMENT = 0.05;

    // good
    public MotorController wheelMotor;
    private boolean wheelSpinning;
    private static final int SHOOTER_ON_SPEED = 418; // inches per second

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
    public Servo pinballServo;
    private double pinballAngle;
    public static final double PINBALL_TURNED = 1;
    public static final double PINBALL_REST = 0;

    public ShooterSystemV1Test(HardwareMap hardwareMap, final LinearOpMode mode) {
        aimServo = new ServoHandler("aimServo", hardwareMap);
        try {
            wheelMotor = new MotorController("wheelMotor", "MotorConfig/NoLoad40.json", hardwareMap);
        } catch (IOException e) {
            e.printStackTrace();
        }
        elevatorServo = new ServoHandler("elevatorServo", hardwareMap);
        elevatorTopSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorTopSwitch"));
        elevatorBottomSwitch = new MagneticLimitSwitch(hardwareMap.digitalChannel.get("elevatorBottomSwitch"));

        pinballServo = hardwareMap.servo.get("pinballServo");

        wheelSpinning = false;
        elevatorPosition = BOTTOM;
        pinballAngle = PINBALL_REST;
        stayAtTop = false;
    }

    public void toggleWheelPower() {
        wheelSpinning = !wheelSpinning;
        wheelMotor.setInchesPerSecondVelocity(wheelSpinning ? SHOOTER_ON_SPEED : 0);
    }

    public void turnOnShooterWheel() {
        wheelSpinning = true;
        wheelMotor.setInchesPerSecondVelocity(SHOOTER_ON_SPEED);
    }

    public void turnOffShooterWheel() {
        wheelSpinning = false;
        wheelMotor.brake();
    }

    // moves the pinball servo
    public void shoot() {
        if (pinballAngle == PINBALL_TURNED)
            pinballAngle = PINBALL_REST;
        else
            pinballAngle = PINBALL_TURNED;

        pinballServo.setPosition(pinballAngle);
    }

    public void raiseShooter() {
        aimServo.setPosition(aimServo.getPosition() - ANGLE_INCREMENT);
    }

    public void lowerShooter() {
        aimServo.setPosition(aimServo.getPosition() + ANGLE_INCREMENT);
    }

    public void setShooter(double angle) { aimServo.setPosition(angle); }

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
    }

    public double calculateRingVelocity(double xDistance, double yDistance) {
        double temp0 = yDistance - xDistance * Math.tan(ConfigVariables.SHOOTER_ANGLE);
        if (temp0 < 0)
            return 0;
        double temp1 = Math.sqrt(-4.9 * xDistance * temp0);
        return Math.cos(ConfigVariables.SHOOTER_ANGLE) / temp1;
    }
}
