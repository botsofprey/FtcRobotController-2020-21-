package Actions.Annie;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import MotorControllers.MotorController;

public class MiscellaneousActions implements ActionHandler {
    HardwareMap hardwareMap;
    private ServoHandler foundationGrabber;
    private MotorController measuringTapeSpitter;
    public static final double FOUNDATION_GRAB = 0, FOUNDATION_RELEASE = 180;

    public MiscellaneousActions(HardwareMap hw) {
        hardwareMap = hw;
        try {
            measuringTapeSpitter = new MotorController("tapeSpitter", "ActionConfig/LiftMotor.json", hardwareMap);
            measuringTapeSpitter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            measuringTapeSpitter.setDirection(DcMotorSimple.Direction.REVERSE);
            measuringTapeSpitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (IOException e) {
            e.printStackTrace();
        }
        foundationGrabber = new ServoHandler("foundationGrabber", hardwareMap);
        foundationGrabber.setServoRanges(0, 180);
        foundationGrabber.setDegree(FOUNDATION_RELEASE);
    }

    public void grabFoundation() { foundationGrabber.setDegree(FOUNDATION_GRAB); }
    public void releaseFoundation(){
        foundationGrabber.setDegree(FOUNDATION_RELEASE);
    }
    public void spitTape() { measuringTapeSpitter.setMotorPower(1); }
    public void retractTape() { measuringTapeSpitter.setMotorPower(-1); }
    public void pauseTape() { measuringTapeSpitter.brake(); }

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
