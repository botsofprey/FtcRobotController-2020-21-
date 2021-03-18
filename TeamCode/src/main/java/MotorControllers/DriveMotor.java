package MotorControllers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.IOException;
import java.io.InputStream;

/**
 * Created by Alex on 3/8/21
 *
 * This is the MotorController class, but it doesn't use any encoders.
 * It is for use with dead wheel encoders so no one uses the encoders and messes up positioning
 */

public class DriveMotor {
	private MotorController motor;
	
	public DriveMotor(DcMotor m, String configFileLoc, HardwareMap hw) throws IOException {
		motor = new MotorController(m, configFileLoc, hw);
	}
	
	public DriveMotor(String motorName, String configFileLoc, HardwareMap hw) throws IOException {
		motor = new MotorController(motorName, configFileLoc, hw);
	}
	
	public DriveMotor(String motorName, String configFileLoc, String debugTag, HardwareMap hw) throws IOException {
		motor = new MotorController(motorName, configFileLoc, debugTag, hw);
	}
	
	public DriveMotor(String motorName, HardwareMap hw) {
		motor = new MotorController(motorName, hw);
	}
	
	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
		motor.setZeroPowerBehavior(b);
	}
	
	public void setDirection(DcMotor.Direction dir) {
		motor.setDirection(dir);
	}
	
	public boolean isBusy() {
		return motor.isBusy();
	}
	
	public void killMotorController() {
		motor.killMotorController();
	}
	
	public double getMotorPower() {
		return motor.getMotorPower();
	}
	
	public double getWheelDiameterInInches() {
		return motor.getWheelDiameterInInches();
	}
	
	public void setMotorPower(double power) {
		motor.setMotorPower(power);
	}
	
	public void brake() {
		motor.brake();
	}
	
	public double getMaxSpeed() {
		return motor.getMaxSpeed();
	}
	
	protected MotorController getMotorController() {
		return motor;
	}
}