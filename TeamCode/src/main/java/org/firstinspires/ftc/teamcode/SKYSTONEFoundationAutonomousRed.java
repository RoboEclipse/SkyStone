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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

@Autonomous(name="SKYSTONEFoundationAutonomousRed", group="Linear Opmode")
//@Disabled
public class SKYSTONEFoundationAutonomousRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private int x;
    // private int y;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        SKYSTONEAutonomousMethods methods = this;
        SKYSTONEClass myRobot = methods.myRobot;
        dashboard = FtcDashboard.getInstance();
        final double speed = 0.75;
        methods.initialize(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addData("Distance", myRobot.getBackDistance() + "");
                    telemetry.update();
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
            TelemetryPacket packet;
            packet = new TelemetryPacket();
            /* packet.put("cat", 3.8);
            //packet.fieldOverlay().setFill("blue").fillRect(-);

            dashboard.sendTelemetryPacket(packet);
            */
            //Raise up foundation servos
            myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
            myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
            //Strafe right to align to foundation
            methods.encoderStrafeDriveInchesRight(SKYSTONEConstants.aFoundationAim, speed);
            //Drive to foundation
            methods.encoderStraightDriveInches(SKYSTONEConstants.bFoundationDistance, speed);
            //Grab foundation
            myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
            myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
            //Drive back with encoder to increase consistency
            methods.encoderStraightDriveInches(-SKYSTONEConstants.bFoundationDistance + 10, speed);
            //Strafe right to ensure the foundation is flush with the wall
            methods.encoderStrafeDriveInchesRight(15, speed);
            //Drive backwards with raw power
            myRobot.runMotors(-0.2, -0.2);
            // Changed from -0.6 to -0.2
            sleep(800);
            // Changed from 1500 t0 800
            myRobot.runMotors(0,0);

            //Lift up foundation servos
            myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
            myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
            sleep(500);
            //Clear the foundation
            methods.encoderStrafeDriveInchesRight(SKYSTONEConstants.cFoundationClear, speed);
            //Drive forward to get off the wall
            //methods.encoderStraightDriveInches(-2, 0.75);
            myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
            myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
            myRobot.runMotors(-0.2, 0.2);
            sleep(300);
            myRobot.runMotors(0,0);
            /*
            methods.encoderStrafeDriveInchesRight(5, speed);
            methods.encoderStraightDriveInches(-30,speed);
            methods.encoderStraightDriveInches(35, speed);
            methods.encoderStrafeDriveInchesRight(-5,speed);
            methods.encoderStrafeDriveInchesRight(5,speed);
            methods.encoderStraightDriveInches(-20,speed);
            */
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
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
