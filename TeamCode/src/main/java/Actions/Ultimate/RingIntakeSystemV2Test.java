package Actions.Ultimate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;

import Actions.ActionHandler;
import Actions.HardwareWrappers.ServoHandler;
import MotorControllers.MotorController;

public class RingIntakeSystemV2Test implements ActionHandler {

    public static final int DEPLOY_ANGLE = 180;
    public static final int INIT_ANGLE = 0;

    private MotorController intakeMotor;
    private ServoHandler intakeServo;

    public RingIntakeSystemV2Test(HardwareMap hardwareMap) {
        try {
            intakeMotor = new MotorController("intakeMotor", "MotorConfig/NoLoad40.json", hardwareMap);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            
            intakeServo = new ServoHandler("intakeServo", hardwareMap);
            intakeServo.setDirection(Servo.Direction.FORWARD);
            intakeServo.setServoRanges(0, 180);

            intakeServo.setDegree(INIT_ANGLE);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    public void drop() {
        intakeServo.setDegree(DEPLOY_ANGLE);
    }

    public void storeIntakeServo() {
        intakeServo.setDegree(INIT_ANGLE);
    }

    public void intake() {
        intakeMotor.setMotorPower(1);
    }

    public void spit() {
        intakeMotor.setMotorPower(-1);
    }

    public void pauseIntake() {
        intakeMotor.brake();
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
        intakeMotor.killMotorController();
    }
}
