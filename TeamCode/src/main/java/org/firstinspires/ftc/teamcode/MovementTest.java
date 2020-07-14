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

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@Autonomous(name="MovementTest", group="Linear Opmode")
//@Disabled
public class MovementTest extends AutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    SKYSTONEDrivetrainClass myRobot = new SKYSTONEDrivetrainClass();

    // private int x;
    // private int y;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        if(dashboard!=null){
            telemetry = dashboard.getTelemetry();
        }

        final double speed = 0.75;
        initializeDrivetrain(hardwareMap, telemetry, this.myRobot);
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        while (!isStarted()) {
            synchronized (this) {
                try {
                    double x = SKYSTONEAutonomousConstants.fieldSize - myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                    double y = myRobot.backDistance.getDistance(DistanceUnit.INCH);
                    localizer.setCoordinates(x, y);
                    Log.d("Skystone", "x: " + localizer.getX());
                    Log.d("Skystone", "y: " + localizer.getY());
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            localizer.useEncoderOnlyToggle(false);
            directionalDrive(SKYSTONEAutonomousConstants.stoneGrabXRed, SKYSTONEAutonomousConstants.stoneGrabY, true,2, 0);
            directionalDrive(SKYSTONEAutonomousConstants.stoneAwayXRed, SKYSTONEAutonomousConstants.stoneAwayY, true,2, 0);
            directionalDrive(144, 28, true,2, 0);
            localizer.useEncoderOnlyToggle(true);
            //localizer.averageDiffs();
            directionalDrive(SKYSTONEAutonomousConstants.stoneGrabXRed, SKYSTONEAutonomousConstants.stoneGrabY, true,2, 0);
            directionalDrive(SKYSTONEAutonomousConstants.stoneAwayXRed, SKYSTONEAutonomousConstants.stoneAwayY, true,2, 0);
            directionalDrive(144, 28, true,2, 0);
            //grabFoundation(true);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize-5, 5, true,0.5, 0);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize-20, 20, true, 0.5, 0);

            /*
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize-10, 20,true,3,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize-5,5,true,3,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize-10, 5,true,3,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize-5,20,true,3,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize-10, 20,true,3,0);

             */
            /*
            encoderStraightDrive(-4,1);
            encoderTurnNoStopPowers(70, -1,-0.25,3);
            encoderTurnNoStopLeftOnly(0,1,3);
            runMotors(0,0);
            */
            //encoderTurnNoStopPowers(70, -1,-0.25,3);
            //encoderTurnNoStopLeftOnly(0,1,3);
            //directionalDrive(5,15, false, 4,0);
            //directionalDrive(15,5, true, 1, 0);
            //encoderStraightDrive(30,1);
            //directionalDrive(15,SKYSTONEAutonomousConstants.fieldSize-15, false, 4,0);
            //sleep(300);
            //directionalDrive(5,SKYSTONEAutonomousConstants.fieldSize-25, true, 1, 0);
            //adaptiveEncoderDrive(40,0,30,1);
            //directionalDrive(15,20, true, 0.5);
            break;
        }
    }

    /*private void dashboardRecordPosition(int deltax, int deltay) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("cat", 3.7);
        packet.fieldOverlay().setFill("blue").fillRect(x,y,x+ deltax,y + deltay +2);

        dashboard.sendTelemetryPacket(packet);
        x = x + deltax;
        y = y + deltay;
    }*/
}
