package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import MotorControllers.MotorController;
import SensorHandlers.LimitSwitch;

/**
 * Author: Ethan Fisher
 * Date: 10/21/2020
 *
 * Used for grabbing and releasing the wobble goal
 */
public class WobbleGrabberV2 implements ActionHandler {

    public ServoHandler leftClaw, rightClaw;
    public MotorController arm;
    public LimitSwitch sensor;

    private static final double ARM_POWER = .35;
    private static final double SLOW_ARM_POWER = 0.175;

    public static final double CLAW_GRAB_POSITION = 1;
    public static final double CLAW_RELEASE_POSITION = -1;

    public static final double ANGLE_INCREMENT = 25;
    public static final double WALL_ANGLE = -40;
    public static final double LIFT_ANGLE = -80;
    public static final double GRAB_AND_DROP_ANGLE = -115;
    public static final double INIT_ANGLE = 0;

    public boolean wobbleGrabbed;

    public WobbleGrabberV2(HardwareMap hardwareMap) {
        leftClaw = new ServoHandler("leftWobbleClaw", hardwareMap);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw = new ServoHandler("rightWobbleClaw", hardwareMap);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        sensor = new LimitSwitch(hardwareMap.touchSensor.get("wobbleGrabberSensor"), "wobbleGrabberSensor");

        try {
            arm = new MotorController("wobbleGrabberArm", "ActionConfig/WobbleArmConfig.json", hardwareMap);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setDefaultTicksPerDegree();
            arm.setTicksPerDegree(arm.getTicksPerDegree() * 3.0/2.0); // 2:3 gear ratio
        } catch (Exception e) {
            e.printStackTrace();
        }
        wobbleGrabbed = false;
    }

    public void setClawGrabAngle() {
        leftClaw.setPosition(CLAW_GRAB_POSITION);
        rightClaw.setPosition(CLAW_GRAB_POSITION);
    }

    public void releaseWobble() {
        leftClaw.setPosition(CLAW_RELEASE_POSITION);
        rightClaw.setPosition(CLAW_RELEASE_POSITION);
    }


    public void setArmAngle(double angle) {
        if(arm.getDegree() < angle){
            arm.setPositionDegrees(angle, ARM_POWER);
        }
        else {
            arm.setPositionDegrees(angle, -ARM_POWER);
        }
    }

    public void setGrabAndDropAngle(){
        setArmAngle(GRAB_AND_DROP_ANGLE);
    }

    public void setLiftAngle(){
        setArmAngle(LIFT_ANGLE);
    }


    public void setWallAngle(){
        setArmAngle(WALL_ANGLE);
    }

    public void setInitAngle(){
        setArmAngle(INIT_ANGLE);
    }

    public void setGrabAngleSlow(){
        arm.setPositionDegrees(GRAB_AND_DROP_ANGLE, SLOW_ARM_POWER);
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
