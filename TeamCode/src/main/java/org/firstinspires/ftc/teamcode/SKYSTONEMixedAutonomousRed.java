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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


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

@Autonomous(name="SKYSTONEMixedAutonomousRed", group="Linear Opmode")
//@Disabled
public class SKYSTONEMixedAutonomousRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //double y = 0;
    FtcDashboard dashboard;
    List<Recognition> updatedRecognitions;
    String skyStonePosition = "Left";
    double dropDistance = SKYSTONEAutonomousConstants.doubleBridgeCross;

    @Override
    public void runOpMode() {

        //SKYSTONEAutonomousMethods methods = this;
        //SKYSTONEClass myRobot = this.myRobot;
        //SKYSTONEVuforiaDetection vuforiaMethods = new SKYSTONEVuforiaDetection();
        dashboard = FtcDashboard.getInstance();
        final double speed = 1;
        initialize(hardwareMap, telemetry);
        //List<VuforiaTrackable> detections = vuforiaMethods.initializeVuforia(hardwareMap);
        //vuforiaMethods.activateDetection();
        myRobot.clawRotation.setPosition(SKYSTONEAutonomousConstants.straight);
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

        while(opModeIsActive()){
            myRobot.leftClaw.setPosition(SKYSTONEAutonomousConstants.flUp);
            myRobot.rightClaw.setPosition(SKYSTONEAutonomousConstants.frUp);
            //Drive the distance
            distanceEncoderDrive(1.5,0.3,1,0, myRobot.frontDistance);
            //Detect where the SkyStone is
            float leftHue = hsv(myRobot.leftColor);
            float rightHue = hsv(myRobot.rightColor);
            if(leftHue >= 100) {
                skyStonePosition = "Left";
                //First Strafe
                encoderStrafeDriveInchesRight(0,1);
                dropDistance+=SKYSTONEAutonomousConstants.doubleAdjustDistance;
            }
            else if(rightHue >= 100) {
                //First Strafe
                encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.doubleAdjustDistance+SKYSTONEAutonomousConstants.doubleCenterDistance+2, 1);
                skyStonePosition  = "Right";
                dropDistance -= SKYSTONEAutonomousConstants.doubleAdjustDistance;
            }
            else {
                //First strafe
                encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.doubleCenterDistance, 1);
                skyStonePosition = "Center";
            }
            Log.d("SkyStone Position: ", skyStonePosition);
            //Grab the stone
            myRobot.leftClaw.setPosition(SKYSTONEAutonomousConstants.flDown);
            sleep(800);
            //Drive backwards
            encoderStraightDriveInches(-4, 1);
            //Turn
            encoderTurn(-88, 1.0, 1);
            //Cross bridge
            encoderStraightDriveInches(dropDistance, 1);
            //Raise up foundation servos
            myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
            myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
            //Drive forward to align with foundation
            encoderStraightDriveInches(SKYSTONEAutonomousConstants.foundationAlign, speed);
            //Turns to match Foundation
            encoderTurn(-178, 1, 2);
            //Drive to foundation
            encoderStraightDriveInches(SKYSTONEAutonomousConstants.foundationDistance, speed);
            foundationPlaceRed(myRobot);
            encoderTurn(88, 1,3);
            //Let go of stone
            myRobot.leftClaw.setPosition(SKYSTONEAutonomousConstants.flUp);
            //Get past skystone so we don't push it
            encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.skystoneClear,1);
            //Drive under bridge
            encoderStraightDriveInches(SKYSTONEConstants.eSkybridge1, 0.6);
            break;
        }
        //vuforiaMethods.deactivateDetection();
        AutoTransitioner.transitionOnStop(this, "SKYSTONETeleOp");
    }
}
