/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package Autonomous.OpModes.UltimateAuto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.AutoAlliance;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigation;

import static Autonomous.ConfigVariables.RING_DETECTION_POINT;
import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_RIGHT;

/*
    Author: Ethan Fisher
    Date: 10/29/2020

    An opmode for the Ultimate Goal Autonomous
 */
@Autonomous(name="UltimateV1Auto", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class UltimateV1AutoRed extends LinearOpMode {

    volatile boolean shouldRun = true;

    @Override
    public void runOpMode() {

        // initialize robot
        Location startLocation = STARTING_ROBOT_LOCATION_RIGHT;
        startLocation.setHeading(UltimateNavigation.SOUTH);
        final UltimateAutonomous robot = new UltimateAutonomous(AutoAlliance.RED, startLocation, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  goes before all of statements in case the 30 seconds is up

        waitForStart();

        robot.getShooter().keepElevatorAtTop();
        robot.getShooter().shoot();

        final long startTime = System.currentTimeMillis();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (shouldRun && opModeIsActive()) {
                    try {
                        robot.getShooter().update();

                        if (!shouldNotPark(startTime)) {
                            robot.park();
                            robot.stop();
                            shouldRun = true;
                        }
                    }
                    catch(Exception e){
                        Log.d("Thread Exception Found:", e.toString());
                    }
                }
            }
        }).start();

         robot.getWobbleGrabber().grabWobbleGoal();

         robot.driveToLocationOnInitHeading(RING_DETECTION_POINT);

         int numRings = robot.detectNumRings();
         telemetry.addData("Rings Found", numRings);
         telemetry.update();
//        if (numRings == 0)
//            robot.driveToLocationOnInitHeading(ringZone);

        robot.driveToRingZone(numRings);

        // move behind shot line, rotate towards goal, and shoot
        robot.moveToShootLocation();
//        robot.getShooter().raiseElevator();
        robot.getShooter().keepElevatorAtTop();
//        robot.shootPowerShots();
        robot.shootThreeRings();
//        if (numRings != 0)
//            robot.grabStartingPileRings();

        // park on the line and stop
        shouldRun = false;
        robot.park();
        robot.stop();
        robot.getShooter().stayAtTop = false;
        robot.getShooter().elevatorServo.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive());
    }
    
    public boolean shouldNotPark(long startTime) {
        long curTimeMillis = System.currentTimeMillis() - startTime;
        double curTimeSeconds = curTimeMillis / 1000.0;
        return opModeIsActive() && curTimeSeconds < 28;
    }

    public boolean isInRange(double first, double second, double tolerance) {
        double difference = first - second;
        difference = Math.abs(difference);
        return difference <= tolerance;
    }
}