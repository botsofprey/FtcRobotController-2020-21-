/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package UserControlled;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.InputStream;

import MotorControllers.JsonConfigReader;
import MotorControllers.MotorController;
import MotorControllers.PIDController;
import SensorHandlers.ImuHandler;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="HolonomicDriveBase", group="Linear Opmode")
//@Disabled
public class HolonomicDriveBase extends LinearOpMode {
    public MotorController[] driveMotors = new MotorController[4];
    public static final int FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR = 0;
    public static final int FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_HOLONOMIC_DRIVE_MOTOR = 3;
    private PIDController headingController, turnController;
    public ImuHandler orientation;
    private HardwareMap hardwareMap;

    private double maxMotorVelocity = 0;

    @Override
    public void runOpMode() {
        String configFile = "MotorConfig\\DriveMotors\\NewHolonomicDriveMotorConfig.json";
        readConfigAndInitialize(configFile);
        double avg = 0;
        for(int i = 0; i < driveMotors.length; i ++){
            avg += driveMotors[i].getMaxSpeed();
        }
        avg /= driveMotors.length;
        maxMotorVelocity = avg;
    }

    public HolonomicDriveBase(HardwareMap hw, String configFile){
        hardwareMap = hw;
        readConfigAndInitialize(configFile);
        double avg = 0;
        for(int i = 0; i < driveMotors.length; i ++){
            avg += driveMotors[i].getMaxSpeed();
        }
        avg /= driveMotors.length;
        maxMotorVelocity = avg;
    }

    public HolonomicDriveBase(HardwareMap hw, double robotOrientationOffset, String configFile){
        this(hw, configFile);
        orientation.setOrientationOffset(robotOrientationOffset);
    }

    /**
     * simplest way to drive the robot. This is not a heading corrected drive
     * @param heading heading relative to the front of the robot
     * @param desiredPower power from 0 to 1 representing power
     */
    public void driveOnHeadingRelativeToRobot(double heading, double desiredPower){
        double [] powers = calculatePowersToDriveOnHeading(heading, desiredPower);
        applyMotorPowers(powers);
    }


    /**
     * way to drive the robot using cartesian driving
     * @param heading the desired heading of the robot with 0 being north
     * @param movementPower power from 0 to 1 representing desired movement power
     * @param turnPower power from -1 to 1 representing desired turning power
     */
    public void cartesianDriveOnHeadingWithTurning(double heading, double movementPower, double turnPower){
        double distanceToHeading = calculateDistanceFromHeading(orientation.getOrientation(),heading);
        Log.d("Dist To Head","" + distanceToHeading);
        driveOnHeadingWithTurning(distanceToHeading, movementPower, turnPower);
    }

    /**
     * way to drive the robot and turn at same time. This is not a heading corrected drive
     * @param heading heading relative to the front of the robot
     * @param movementPower power from 0 to 1 representing the movement speed
     * @param turnPower power from -1 to 1 representing the turning speed
     */
    public void driveOnHeadingWithTurning(double heading, double movementPower, double turnPower){
        double [] movementPowers = calculatePowersToDriveOnHeading(heading, movementPower);
        double [] turningPowers = calculatePowersToTurn(turnPower);
        double [] total = new double[4];
        for(int i = 0; i < movementPowers.length; i ++){
            total[i] = movementPowers[i] + turningPowers[i];
        }
        normalizePowers(total);
        applyMotorPowers(total);
    }

    /**
     * turns the robot at the desired proportion of max
     * @param percentOfMaxTurnRate value from -1 to 1 corresponding to the max turn rate
     */
    public void turn(double percentOfMaxTurnRate) {
        double[] velocities = calculatePowersToTurn(percentOfMaxTurnRate);
        applyMotorVelocities(velocities);
    }


    /**
     * brakes the robot
     */
    public void brake(){
        applyMotorPowers(new double[] {0,0,0,0});
    }

    /**
     * normalizes all powers to -1 and 1, scales appropriately
     * @param toNormalize an array of doubles that have a wanted value of -1 to 1
     */
    private void normalizePowers(double [] toNormalize){
        //get the min and max powers
        double min = toNormalize[0], max = toNormalize[0];
        for(int i = 0; i < toNormalize.length; i ++){
            if(toNormalize[i] < min) min = toNormalize[i];
            else if(toNormalize[i] > max) max = toNormalize[i];
        }
        //assign toScaleAgainst to the largest (abs) value
        double toScaleAgainst = 0;
        if(Math.abs(min) < Math.abs(max)) toScaleAgainst = Math.abs(max);
        else toScaleAgainst = Math.abs(min);
        //if the largest (abs) is greater than 1, scale all values appropriately
        if(toScaleAgainst > 1){
            for(int i = 0; i < toNormalize.length; i ++){
                toNormalize[i] = toNormalize[i]/toScaleAgainst;
            }
        }
    }

    /**
     * calculates the powers required to make the robot move on a heading
     * @param heading from 0 to 360, represents the desired heading of the robot relative to the front of the robot
     * @param desiredPower from 0 to 1 that represents the desired proportion of max velocity for the robot to move at
     * @return a double array with the calculated motor powers for each wheel
     */
    private double [] calculatePowersToDriveOnHeading(double heading, double desiredPower){
        double[] powers = new double[4];
        if(desiredPower == 0){
            for(int i = 0; i < powers.length; i ++){
                powers[i] = 0;
            }
            return powers;
        }
        powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.sin(Math.toRadians(heading + 45));
        powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.cos(Math.toRadians(heading + 45));
        powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.sin(Math.toRadians(heading + 45));
        powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredPower * Math.cos(Math.toRadians(heading + 45));
        //Log.d("MotorPow","" + powers[0]);
        return powers;
    }

    /**
     * calculates the powers of each motor to turn at the desired rate
     * @param desiredTurnRateOfMax a value from -1 to 1 that represents the rate of max turn to turn at
     * @return a double array with the calculated motor powers for each wheel
     */
    private double [] calculatePowersToTurn(double desiredTurnRateOfMax){
        double[] powers = new double[4];
        if(desiredTurnRateOfMax == 0){
            for(int i = 0; i < powers.length; i ++){
                powers[i] = 0;
            }
            return powers;
        }
        powers[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredTurnRateOfMax;
        powers[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -desiredTurnRateOfMax;
        powers[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = desiredTurnRateOfMax;
        powers[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = -desiredTurnRateOfMax;
        for(int i = 0; i < powers.length; i ++){
            if(Double.isNaN(powers[i])) powers[i] = 0;
        }
        return powers;
    }

    /**
     * sets each motor at the desired motor power, -1 to 1 corresponding to max velocities
     * @param powers a double array of length 4 with values -1 to 1
     */
    private void applyMotorPowers(double [] powers){
        normalizePowers(powers);
        double [] velocities = new double[4];
        for(int i = 0; i < powers.length; i ++){
            //Log.d("Motor " + i + "Power", "" + powers[i]);
            velocities[i] = powers[i]*maxMotorVelocity;
        }
        applyMotorVelocities(velocities);
    }

    /**
     * sets each motor to turn at the desired velocity
     * @param velocities a double array of length 4 with the desired motor velocities to set the motors at
     */
    private void applyMotorVelocities(double [] velocities){
        driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR]);
        driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setInchesPerSecondVelocity(velocities[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR]);
    }


    /**
     * calculates the distance between two angle. For example, if curAngle was 350 deg and target was 45 deg, 55 degs would be returned
     * @param curAngle current or base angle to compare from. Should be a value from 0 to 360
     * @param targetAngle angle to compare to, should be a value from 0 to 360;
     * @return the distance from the curAngle to the target angle. Is + if target angle is to the Right, - if to the left
     */
    public double calculateDistanceFromHeading(double curAngle, double targetAngle){
        double distanceFromHeading = targetAngle - curAngle;
        return normalizeAngle(distanceFromHeading);
    }


    /**
     * normalizes a degree to a value between -180 and 180
     * @param angle angle to normalize, can be any degree
     * @return the normalized angle value from -180 to 180
     */
    private double normalizeAngle(double angle){
        angle %= 360;
        if(angle > 180) angle = 360 - angle;
        else if(angle < -180) angle += 360;
        return angle;
    }


    /**
     * kills all parts of the robot for a safe shutdown
     */
    public void kill(){
        for (MotorController driveMotor : driveMotors) {
            driveMotor.killMotorController();
        }
        orientation.stopIMU();
    }

    /**
     * reads the robot config file and initializes all motors using applicable data
     * @param file the location of the robot config file, in the assets folder
     */
    private void readConfigAndInitialize(String file){
        InputStream stream = null;
        try {
            stream = hardwareMap.appContext.getAssets().open(file);
        }
        catch(Exception e){
            Log.d("Drive Engine Error: ",e.toString());
            throw new RuntimeException("Drive Engine Open Config File Fail: " + e.toString());
        }
        JsonConfigReader reader = new JsonConfigReader(stream);
        try{
            driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("FRONT_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("FRONT_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("BACK_LEFT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR] = new MotorController(reader.getString("BACK_RIGHT_MOTOR_NAME"), "MotorConfig/DriveMotors/NewHolonomicDriveMotorConfig.json", hardwareMap);
            for (int i = 0; i < driveMotors.length; i++) {
                driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("BRAKE")){
                for (int i = 0; i < driveMotors.length; i++) {
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else if(reader.getString("DRIVE_MOTOR_BRAKING_MODE").equals("FLOAT")){
                for (int i = 0; i < driveMotors.length; i++) {
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            if(reader.getString("FRONT_LEFT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("FRONT_LEFT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[FRONT_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("FRONT_RIGHT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("FRONT_RIGHT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[FRONT_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("BACK_RIGHT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("BACK_RIGHT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[BACK_RIGHT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if(reader.getString("BACK_LEFT_MOTOR_DIRECTION").equals("REVERSE")) {
                driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else if(reader.getString("BACK_LEFT_MOTOR_DIRECTION").equals("FORWARD")) {
                driveMotors[BACK_LEFT_HOLONOMIC_DRIVE_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
            }
            headingController = new PIDController(reader.getDouble("HEADING_Kp"), reader.getDouble("HEADING_Ki"), reader.getDouble("HEADING_Kd"));
            headingController.setIMax(reader.getDouble("HEADING_Ki_MAX"));
            turnController = new PIDController(reader.getDouble("TURN_Kp"), reader.getDouble("TURN_Ki"), reader.getDouble("TURN_Kd"));
            turnController.setIMax(reader.getDouble("TURN_Ki_MAX"));
        } catch(Exception e){
            Log.e(" Drive Engine Error", "Config File Read Fail: " + e.toString());
            throw new RuntimeException("Drive Engine Config Read Failed!:" + e.toString());
        }
    }
}
