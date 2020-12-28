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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Autonomous.AutoAlliance;
import Autonomous.Location;
import DriveEngine.Ultimate.UltimateNavigation;

import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;
import static Autonomous.ConfigVariables.STARTING_ROBOT_LOCATION_RIGHT;

/*
    Author: Ethan Fisher
    Date: 10/29/2020

    An opmode for the Ultimate Goal Autonomous
 */
@Autonomous(name="Ultimate Auto North Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class UltimateAutoNorthTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // initialize robot
        Location startLocation = STARTING_ROBOT_LOCATION_RIGHT;
        startLocation.setHeading(UltimateNavigation.NORTH);
        final UltimateAutonomous robot = new UltimateAutonomous(AutoAlliance.RED, startLocation, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // if (shouldNotPark(startTime)) goes before all of statements in case the 30 seconds is up

        waitForStart();

        final long startTime = System.currentTimeMillis();
        
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (!shouldNotPark(startTime)) {
                        robot.park();
                        robot.stop();
                    }
                }
            }
        }).start();
        
//        robot.getShooter().keepElevatorAtTop();
        robot.getShooter().shoot();

        robot.getWobbleGrabber().grabWobbleGoal();
        robot.getWobbleGrabber().lowerArm();
        robot.driveToRingCheckpoint();
        
        int numRings = robot.detectNumRings();
        Location ringZone = robot.getZone(numRings);
        telemetry.addData("Rings Found", numRings);
        telemetry.update();
        
        robot.driveToLocationOnInitHeading(SHOOTING_LINE_POINT);
        robot.shootThreeRings();
        
        robot.driveToLocationOnInitHeading(ringZone);
        robot.dropWobbleGoal();
        robot.getWobbleGrabber().raiseArm();
        
        robot.driveToLeftWobbleGoalAndGrab();
        robot.robot.turnToHeading(UltimateNavigation.SOUTH, this);
        robot.pickupWobbleGoal();
        robot.driveToLocationOnHeading(ringZone, UltimateNavigation.SOUTH);
        robot.dropWobbleGoal();

        // TODO pick up starting pile

        // park on the line and stop
        robot.park();
        robot.stop();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive());
    }
    
    public boolean shouldNotPark(long startTime) {
        long curTimeMillis = System.currentTimeMillis() - startTime;
        double curTimeSeconds = curTimeMillis / 1000.0;
        return opModeIsActive() && curTimeSeconds < 28;
    }
}

