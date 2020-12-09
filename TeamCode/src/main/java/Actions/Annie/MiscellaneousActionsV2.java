package Actions.Annie;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import MotorControllers.MotorController;

public class MiscellaneousActionsV2 implements ActionHandler {
    HardwareMap hardwareMap;
    private ServoHandler leftFoundationGrabber, rightFoundationGrabber;
    private MotorController measuringTapeSpitter;
    public static final double FOUNDATION_GRAB = 0, FOUNDATION_RELEASE = 180;

    public MiscellaneousActionsV2(HardwareMap hw) {
        hardwareMap = hw;
        try {
            measuringTapeSpitter = new MotorController("tapeSpitter", "ActionConfig/LiftMotor.json", hardwareMap);
            measuringTapeSpitter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            measuringTapeSpitter.setDirection(DcMotorSimple.Direction.REVERSE);
            measuringTapeSpitter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (IOException e) {
            e.printStackTrace();
        }
        leftFoundationGrabber = new ServoHandler("leftFoundationGrabber", hardwareMap);
        leftFoundationGrabber.setServoRanges(0, 180);
        leftFoundationGrabber.setDegree(FOUNDATION_RELEASE);
        rightFoundationGrabber = new ServoHandler("rightFoundationGrabber", hardwareMap);
        rightFoundationGrabber.setServoRanges(0, 180);
        rightFoundationGrabber.setDirection(Servo.Direction.REVERSE);
        rightFoundationGrabber.setDegree(FOUNDATION_RELEASE);
    }

    public void grabFoundation() {
        rightFoundationGrabber.setDegree(FOUNDATION_GRAB);
        leftFoundationGrabber.setDegree(FOUNDATION_GRAB);
    }
    public void releaseFoundation() {
        rightFoundationGrabber.setDegree(FOUNDATION_RELEASE);
        leftFoundationGrabber.setDegree(FOUNDATION_RELEASE);
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
