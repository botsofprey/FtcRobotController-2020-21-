package Actions.HardwareWrappers;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

import MotorControllers.MotorController;
import MotorControllers.PIDController;

public class WheelMotor {

    public MotorController motor;
    public volatile double curRPM;
    public volatile double targetRPM;
    private long prevTicks;
    private long prevTime;

    private static final int MAX_RPM = 5400;
    private static final int RPM_LIMIT = 3700;
    private static final double MINIMUM_TIME_DIFFERENCE = 100000000;// 1/10 of a second
    private static final long NANOS_PER_MINUTE = 60000000000L;
    private static final double TICKS_PER_ROTATION = 28;
    
    private static final int RPM_TOLERANCE = 150;
    
    private static final double KP = 1.4;
    private static final double KI = 0;
    private static final double KD = 0.2;

    private PIDController rpmController;

    private RevBlinkinLedDriver ledController;
    
    public WheelMotor(String name, HardwareMap hardwareMap) {
        motor = new MotorController(name, hardwareMap);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetRPM = 0;
        prevTime = System.nanoTime();
        prevTicks = motor.getCurrentTick();
        
        rpmController = new PIDController(1.4, 0, .2);
    }
    
    public WheelMotor(String name, String ledControllerName, HardwareMap hardwareMap) {
        motor = new MotorController(name, hardwareMap);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetRPM = 0;
        prevTime = System.nanoTime();
        prevTicks = motor.getCurrentTick();
//        ledController = hardwareMap.get(RevBlinkinLedDriver.class, ledControllerName);
        
        rpmController = new PIDController(KP, KI, KD);
    }

    public void setRPM(int RPM) {
        rpmController.setSp(targetRPM = Math.min(RPM, RPM_LIMIT));
    }

    public void updateShooterRPM() {
        long currentTicks = motor.getCurrentTick();
        long currentTime = System.nanoTime();

        double tickDiff = currentTicks - prevTicks;
        double timeDiff = currentTime - prevTime;
        if (timeDiff > MINIMUM_TIME_DIFFERENCE) {
            curRPM = (tickDiff / timeDiff) * (NANOS_PER_MINUTE / TICKS_PER_ROTATION);
            prevTicks = currentTicks;
            prevTime = currentTime;
            adjustRPM();
            Log.d("RPM", "" + curRPM);
//            if (Math.abs(curRPM - targetRPM) < RPM_TOLERANCE) {
//                ledController.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//            }
//            else {
//                ledController.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            }
        }
    }

    private void adjustRPM() {
        double rpmCorrection = rpmController.calculatePID(curRPM);
        if (targetRPM == 0) {
            motor.setMotorPower(0);
            return;
        }
        motor.setMotorPower(rpmCorrection / MAX_RPM + motor.getMotorPower());

//        double curRPM = (dTicks / (double) dt) * (NANOS_PER_MINUTE / TICKS_PER_ROTATION);
//        double rpmCorrection = rpmController.calculatePID(curRPM);

//        motor.setPower((curRPM + rpmCorrection) / MAX_RPM);

//        mode.telemetry.addData("Motor Power", "" + targetPower);
//        mode.telemetry.update();
    }
}