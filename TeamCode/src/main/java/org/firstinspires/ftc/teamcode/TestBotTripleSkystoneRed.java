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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="TestBotTripleSkystoneRed", group="Linear Opmode")
//@Disabled
public class TestBotTripleSkystoneRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private int x;
    // private int y;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        final double speed = 0.75;
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        while (!isStarted()) {
            synchronized (this) {
                try {
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
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+10.0, true, 2,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 20, 8.0/3+15.0, false, 5,0);
            straighteningEncoderDrive(-65, 0, 50, 1);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, SKYSTONEAutonomousConstants.fieldSize-5, false, 2,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 20, SKYSTONEAutonomousConstants.fieldSize-10, false, 2,0);
            straighteningEncoderDrive(72, 0, 50, 1);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 16.0/3+10.0, true, 2,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 20, 16.0/3+15.0, false, 5,0);
            straighteningEncoderDrive(-65, 0, 50, 1);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, SKYSTONEAutonomousConstants.fieldSize-13, false, 2,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 20, SKYSTONEAutonomousConstants.fieldSize-18, false, 2,0);
            straighteningEncoderDrive(72, 0, 50, 1);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8+10.0, true, 2,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 20, 8+15.0, false, 5,0);
            straighteningEncoderDrive(-65, 0, 50, 1);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, SKYSTONEAutonomousConstants.fieldSize-21, false, 2,0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 20, SKYSTONEAutonomousConstants.fieldSize-26, false, 2,0);
            /*
            grabAndPlace(8.0/3+10, 20);
            encoderStraightDrive(80, 1);
            grabAndPlace(16.0/3+10, 14);
            encoderStraightDrive(80, 1);
            grabAndPlace(8+10, 8);
            encoderTurn(90,1,3);

             */
            break;
        }
    }

    private void grabAndPlace(double closeWallDistance, double farWallDistance) {
        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 8, closeWallDistance, true, 2.5,0);
        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 3,(closeWallDistance-4), true, 2.5,0);
        encoderStraightDrive(-50, 1);
        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 8, SKYSTONEAutonomousConstants.fieldSize-farWallDistance, true, 2.5, 0);
        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 3, SKYSTONEAutonomousConstants.fieldSize-(farWallDistance-4), true, 2.5, 0);
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
