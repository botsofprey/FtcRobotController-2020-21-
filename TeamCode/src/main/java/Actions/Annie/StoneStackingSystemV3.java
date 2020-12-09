package Actions.Annie;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Actions.ActionHandler;
import Actions.HardwareWrappers.DoubledSpoolMotor;
import Actions.HardwareWrappers.ServoHandler;

import static Actions.HardwareWrappers.DoubledSpoolMotor.MOTOR_1;
import static Actions.HardwareWrappers.DoubledSpoolMotor.MOTOR_2;

public class StoneStackingSystemV3 implements ActionHandler {

    HardwareMap hardwareMap;
    DoubledSpoolMotor lift;
    ServoHandler centralGripper, capstoneDeployer;
    public static final double CENTRAL_ARM_GRAB = 0, CENTRAL_ARM_RELEASE = 65, CENTRAL_ARM_IN_BOT = 180;
    public static final int STONE_HEIGHT_RESET = -1, STONE_HEIGHT_1 = 0, STONE_HEIGHT_2 = 1, STONE_HEIGHT_3 = 2, STONE_HEIGHT_4 = 3, STONE_HEIGHT_5 = 4;
    public static final int MOTOR_1_HEIGHT_0 = 0, MOTOR_1_HEIGHT_1 = 923, MOTOR_1_HEIGHT_2 = 1970, MOTOR_1_HEIGHT_3 = 2785, MOTOR_1_HEIGHT_4 = 3622, MOTOR_1_HEIGHT_5 = 4665;
    public static final int MOTOR_2_HEIGHT_0 = 0, MOTOR_2_HEIGHT_1 = 1022, MOTOR_2_HEIGHT_2 = 1972, MOTOR_2_HEIGHT_3 = 2733, MOTOR_2_HEIGHT_4 = 3592, MOTOR_2_HEIGHT_5 = 4586;

    public StoneStackingSystemV3(HardwareMap hw) {
        hardwareMap = hw;
        lift = new DoubledSpoolMotor(new String[] {"liftMotor1", "liftMotor2"}, "ActionConfig/SSSLift.json", 50, 50, hardwareMap);
        centralGripper = new ServoHandler("centralGripper", hardwareMap);
        capstoneDeployer = new ServoHandler("capStoneDeployer", hardwareMap);
        centralGripper.setServoRanges(0, 180);
        capstoneDeployer.setServoRanges(0, 180);
        centralGripper.setDegree(CENTRAL_ARM_IN_BOT);
        capstoneDeployer.setDegree(0);
//        winchServo.setDegree(0); // 30 degrees in one turn on this servo
//        measureTapeSpitter.
    }

//    public void setWinchServoDegree(double degree) {winchServo.setDegree(degree);}

    public void deployCapstone() { capstoneDeployer.setDegree(180); }
    public void releaseCapstone() { capstoneDeployer.setDegree(0); }
    public void setCapstoneDegree(double degree) {capstoneDeployer.setDegree(degree); }

    public void setCentralGripperDegree(double deg) {
        centralGripper.setDegree(deg);
    }
    public void grabStoneCenter() {
        centralGripper.setDegree(CENTRAL_ARM_GRAB);
    }
    public void releaseStoneCenter() {
        centralGripper.setDegree(CENTRAL_ARM_RELEASE);
    }

    public void liftStones() {
        lift.extend();
    }
    public void lowerStones() { lift.retract(); }
    public void pauseStoneLift() {
        lift.holdPosition();
    }
    public void setLiftPosition(int posInInches) {
        // THESE ARE REDUNDANT, USED IN MOTORCONTROLLER CLASS
        lift.setPower(0);
        lift.setMode(MOTOR_1, DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(MOTOR_2, DcMotor.RunMode.RUN_TO_POSITION);

        switch (posInInches) {
            case STONE_HEIGHT_RESET:
                lift.setPostitionTicks(MOTOR_1, MOTOR_1_HEIGHT_0);
                lift.setPostitionTicks(MOTOR_2, MOTOR_2_HEIGHT_0);
                break;
            case STONE_HEIGHT_1:
                lift.setPostitionTicks(MOTOR_1, MOTOR_1_HEIGHT_1);
                lift.setPostitionTicks(MOTOR_2, MOTOR_2_HEIGHT_1);
                break;
            case STONE_HEIGHT_2:
                lift.setPostitionTicks(MOTOR_1, MOTOR_1_HEIGHT_2);
                lift.setPostitionTicks(MOTOR_2, MOTOR_2_HEIGHT_2);
                break;
            case STONE_HEIGHT_3:
                lift.setPostitionTicks(MOTOR_1, MOTOR_1_HEIGHT_3);
                lift.setPostitionTicks(MOTOR_2, MOTOR_2_HEIGHT_3);
                break;
            case STONE_HEIGHT_4:
                lift.setPostitionTicks(MOTOR_1, MOTOR_1_HEIGHT_4);
                lift.setPostitionTicks(MOTOR_2, MOTOR_2_HEIGHT_4);
                break;
            case STONE_HEIGHT_5:
                lift.setPostitionTicks(MOTOR_1, MOTOR_1_HEIGHT_5);
                lift.setPostitionTicks(MOTOR_2, MOTOR_2_HEIGHT_5);
                break;
        }
        lift.setPower(1);
    }
    public void liftToPosition(int pos) {
        switch (pos) {
            case 0:
                setLiftPosition(STONE_HEIGHT_RESET);
                break;
            case 1:
                setLiftPosition(STONE_HEIGHT_1);
                break;
            case 2:
                setLiftPosition(STONE_HEIGHT_2);
                break;
            case 3:
                setLiftPosition(STONE_HEIGHT_3);
                break;
            case 4:
                setLiftPosition(STONE_HEIGHT_4);
                break;
            case 5:
                setLiftPosition(STONE_HEIGHT_5);
                break;
            default:
                break;
        }
    }
    public double getLiftPositionInches(int motor) { return lift.getPositionInches(motor); }
    public long getLiftPositionTicks(int motor) { return lift.getPosition(motor); }
    public void resetLiftEncoder() {
        lift.setPower(0);
        lift.setMode(MOTOR_1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(MOTOR_2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    }
}
