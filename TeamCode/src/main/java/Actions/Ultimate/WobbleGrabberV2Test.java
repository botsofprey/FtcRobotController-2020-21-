package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import MotorControllers.MotorController;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV2Test implements ActionHandler {

    public ServoHandler claw;
    public MotorController arm;

    private static final double ARM_POWER_DOWN = -.2;
    private static final double ARM_POWER_UP = .25;

    public static final double CLAW_GRAB_ANGLE = 0.0;
    public static final double CLAW_RELEASE_ANGLE = .9;
    public static final double GAINS_ANGLE = 0;
    public static final double LIFT_ANGLE = 0;
    public static final double GRAB_AND_DROP_ANGLE = 0;
    public static final double INIT_ANGLE = 0;

    public boolean wobbleGrabbed;

    public WobbleGrabberV2Test(HardwareMap hardwareMap) {
        claw = new ServoHandler("wobbleGrabberClaw", hardwareMap);
        claw.setDirection(Servo.Direction.FORWARD);

        try {
            arm = new MotorController("wobbleGrabberArm", "ActionConfig/WobbleArmConfig.json", hardwareMap);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            e.printStackTrace();
        }

        wobbleGrabbed = false;
    }

    public void grabWobble() {
        claw.setPosition(CLAW_GRAB_ANGLE);
    }

    public void releaseWobble() {
        claw.setPosition(CLAW_RELEASE_ANGLE);
    }

    public void setArmAngle(double angle) {
        arm.setPositionDegrees(angle);
        arm.setMotorPower(ARM_POWER_UP);
    }

    public void grabWobbleGoal() {
        claw.setPosition(CLAW_GRAB_ANGLE);
    }

    public void pause() {
        arm.brake();
    }

    public boolean armIsBusy() {
        return arm.isBusy();
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
        arm.killMotorController();
    }
}
