package MotorControllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Alex on 3/8/21
 *
 * This class is for interfacing with odometry wheels.
 * Using a separate class helps separate the encoders and drive motors.
 */

public class OdometryWheel {
	private MotorController encoder;
	
	public OdometryWheel(MotorController encoder) {
		this.encoder = encoder;
	}
	
	public long getCurrentTicksPerSecond() {
		return encoder.getCurrentTicksPerSecond();
	}
	
	public double getCurrentInchesPerSecond() {
		return encoder.getCurrentInchesPerSecond();
	}
	
	public double getCurrentRPS() {
		return encoder.getCurrentRPS();
	}
	
	public int getCurrentRPM() {
		return encoder.getCurrentRPM();
	}
	
	public long getCurrentTick() {
		return encoder.getCurrentTick();
	}
	
	public double getInchesFromStart() {
		return encoder.getInchesFromStart();
	}
	
	public double getDegree() {
		return encoder.getDegree();
	}
	
	public double convertTicksToInches(long ticks){
		return encoder.convertTicksToInches(ticks);
	}
	
	public double getWheelDiameterInInches() {
		return encoder.getWheelDiameterInInches();
	}
	
	public int getTicksPerRevolution() {
		return encoder.getTicksPerRevolution();
	}
	
	public double getMaxSpeed() {
		return encoder.getMaxSpeed();
	}
	
	public double getTicksPerDegree() {
		return encoder.getTicksPerDegree();
	}
}
