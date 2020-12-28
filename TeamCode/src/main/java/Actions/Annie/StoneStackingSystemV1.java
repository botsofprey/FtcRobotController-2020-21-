package Actions.Annie;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import Actions.HardwareWrappers.SpoolMotor;
import MotorControllers.MotorController;

public class StoneStackingSystemV1 implements ActionHandler {

    HardwareMap hardwareMap;
    SpoolMotor lift;
    MotorController leftRake, rightRake;
    ServoHandler leftRakeArm, rightRakeArm, centralGripper, levatronArm;
    public static final double LEFT_ARM_STOP = 0.5, RIGHT_ARM_STOP = 0.5, LEFT_ARM_DEPLOY = 1, RIGHT_ARM_DEPLOY = 1, LEFT_ARM_RAISE = 0, RIGHT_ARM_RAISE = 0;
    public static final double CENTRAL_ARM_GRAB = 39, CENTRAL_ARM_RELEASE = 169, LEVATRON_SET = 0, LEVATRON_RELEASE = 180;

    public StoneStackingSystemV1(HardwareMap hw) {
        hardwareMap = hw;
        try {
            lift = new SpoolMotor(new MotorController("lift", "ActionConfig/SSSLift.json", hardwareMap), 50, 50, 35, hardwareMap);
            leftRake = new MotorController("leftRake", "ActionConfig/RakeMotor.json", hardwareMap);
            rightRake = new MotorController("rightRake", "ActionConfig/RakeMotor.json", hardwareMap);

            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRake.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IOException e) {
            e.printStackTrace();
        }
        leftRakeArm = new ServoHandler("leftRakeArm", hardwareMap);
        rightRakeArm = new ServoHandler("rightRakeArm", hardwareMap);
        centralGripper = new ServoHandler("centralGripper", hardwareMap);
        levatronArm = new ServoHandler("levatron", hardwareMap);
        leftRakeArm.setServoRanges(1, 179);
        rightRakeArm.setServoRanges(1, 179);
        centralGripper.setServoRanges(1, 179);
        levatronArm.setServoRanges(1, 179);
        leftRakeArm.setPosition(LEFT_ARM_STOP);
        rightRakeArm.setPosition(RIGHT_ARM_STOP);
        centralGripper.setDegree(CENTRAL_ARM_RELEASE);
        levatronArm.setDegree(LEVATRON_SET);
    }

    public void stopArms() {
        leftRakeArm.setPosition(LEFT_ARM_STOP);
        rightRakeArm.setPosition(RIGHT_ARM_STOP);
    }
    public void deployArms() {
        leftRakeArm.setPosition(LEFT_ARM_DEPLOY);
        rightRakeArm.setPosition(RIGHT_ARM_DEPLOY);
    }
    public void liftArms() {
        leftRakeArm.setPosition(LEFT_ARM_RAISE);
        rightRakeArm.setPosition(RIGHT_ARM_RAISE);
    }
    public void setLeftArmPosition(double pos) {
        leftRakeArm.setPosition(pos);
    }
    public void setRightArmPosition(double pos) {
        rightRakeArm.setPosition(pos);
    }

    public void setCentralGripperDegree(double deg) {
        centralGripper.setDegree(deg);
    }
    public void grabStoneCenter() {
        centralGripper.setDegree(CENTRAL_ARM_GRAB);
    }
    public void dropCapStone() {
        levatronArm.setDegree(LEVATRON_RELEASE);
    }
    public void resetCapStone() {
        levatronArm.setDegree(LEVATRON_SET);
    }
    public void releaseStoneCenter() {
        centralGripper.setDegree(CENTRAL_ARM_RELEASE);
    }

    public void liftStones() {
        lift.extend();
    }
    public void lowerStones() {
        lift.retract();
    }
    public void pauseStoneLift() {
        lift.pause();
    }
    public void liftToPosition(int pos) {
        switch (pos) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                break;
        }
    }
    public void setLiftPosition(double posInInches) {
        lift.setPostitionInches(posInInches);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }
    public double getLiftPositionInches() { return lift.getPositionInches(); }

    public void extendLeftArm() { leftRake.setMotorPower(1); }
    public void retractLeftArm() {
        leftRake.setMotorPower(-1);
    }
    public void pauseLeftArm() {
        leftRake.brake();
    }

    public void extendRightArm() {
        rightRake.setMotorPower(1);
    }
    public void retractRightArm() {
        rightRake.setMotorPower(-1);
    }
    public void pauseRightArm() { rightRake.brake(); }

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

    }
}
