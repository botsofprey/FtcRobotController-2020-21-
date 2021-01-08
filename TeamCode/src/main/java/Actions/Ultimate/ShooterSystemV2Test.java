package Actions.Ultimate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public ServoHandler pinballServo;
    private double pinballAngle;
    public static final double PINBALL_TURNED = 1;
    public static final double PINBALL_REST = 0;

    public ShooterSystemV2Test(HardwareMap hardwareMap) {
        wheelMotor = new WheelMotor("wheelMotor", hardwareMap);

        pinballServo = new ServoHandler("pinballServo", hardwareMap);
        pinballServo.setDirection(Servo.Direction.FORWARD);

        wheelSpinning = false;
        pinballAngle = PINBALL_REST;
    }

    public void turnOnShooterWheel() {
        wheelMotor.setRPM(SHOOTER_ON_SPEED);
    }

    public void turnOffShooterWheel() {
        wheelMotor.setRPM(SHOOTER_OFF_SPEED);
    }
    
    public void toggleShooterWheel() {
        if (wheelMotor.targetRPM == 0)
            wheelMotor.setRPM(SHOOTER_ON_SPEED);
        else
            wheelMotor.setRPM(SHOOTER_OFF_SPEED);
    }

    // moves the pinball servo
    public void togglePinball() {
        if (pinballAngle == PINBALL_REST) {
            pinballAngle = PINBALL_TURNED;
        }
        else {
            pinballAngle = PINBALL_REST;
        }

        pinballServo.setPosition(pinballAngle);
    }
    
    public void setPowerShotSpeed() {
        wheelMotor.setRPM(POWER_SHOT_SPEED);
    }
    
    public void setHighGoalSpeed() {
        wheelMotor.setRPM(HIGH_GOAL_SPEED);
    }

    public void update() { wheelMotor.updateShooterRPM(); }

    public double calculateRingVelocity(double xDistance, double yDistance) {
        double temp0 = yDistance - xDistance * Math.tan(ConfigVariables.SHOOTER_ANGLE);
        if (temp0 < 0)
            return 0;
        double temp1 = Math.sqrt(-4.9 * xDistance * temp0);
        return Math.cos(ConfigVariables.SHOOTER_ANGLE) / temp1;
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
