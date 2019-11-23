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

@Autonomous(name="SKYSTONEDoubleAutonomousRed", group="Linear Opmode")
//@Disabled
public class SKYSTONEDoubleAutonomousRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //double y = 0;
    FtcDashboard dashboard;
    List<Recognition> updatedRecognitions;
    String skyStonePosition = "Left";

    @Override
    public void runOpMode() {

        //SKYSTONEAutonomousMethods methods = this;
        //SKYSTONEClass myRobot = this.myRobot;
        dashboard = FtcDashboard.getInstance();
        final double speed = 1;
        initialize(hardwareMap, telemetry);
        myRobot.clawRotation.setPosition(SKYSTONEConstants.straight);
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
            myRobot.leftClaw.setPosition(0.3);
            myRobot.rightClaw.setPosition(0.3);
            //Drive the distance
            distanceEncoderDrive(1,1,1,0, myRobot.frontDistance);
            float leftHue = hsv(myRobot.leftColor);
            float rightHue = hsv(myRobot.rightColor);
            if(leftHue >= 115) {
                skyStonePosition = "Left";
            }
            else if(rightHue >= 115) {
                skyStonePosition  = "Right";
            }
            else {
                skyStonePosition = "Center";
            }
            if(skyStonePosition.equals("Left")) {
                encoderStrafeDriveInchesRight(-SKYSTONEConstants.doubleAdjustDistance,1);
            }
            if(skyStonePosition.equals("Right")) {
                encoderStrafeDriveInchesRight(SKYSTONEConstants.doubleAdjustDistance, 1);
            }
            //Grab the stone
            myRobot.leftClaw.setPosition(1);
            myRobot.rightClaw.setPosition(1);
            sleep(800);
            //Drive backwards
            encoderStraightDriveInches(-8, 1);
            //Turn
            encoderTurn(-90, 1.0, 3);
            //Drive forwards
            double dropDistance = SKYSTONEConstants.doubleBridgeCross;
            double wallDistance = SKYSTONEConstants.doubleWallDistance;
            if(skyStonePosition.equals("Left")){
                dropDistance+=SKYSTONEConstants.doubleAdjustDistance;
                wallDistance -= 8;
            }
            if(skyStonePosition.equals("Right")){
                dropDistance+=SKYSTONEConstants.doubleAdjustDistance;
                wallDistance += 8;
            }
            encoderStraightDriveInches(dropDistance, 1);
            //Release the stone
            myRobot.leftClaw.setPosition(0.3);
            //myRobot.rightClaw.setPosition(0.3);
            sleep(800);
            //Drive backwards
            distanceEncoderDrive(wallDistance,1,-1,-90, myRobot.backDistance);
            //encoderStraightDriveInches(-dropDistance - 3*SKYSTONEConstants.doubleAdjustDistance, 1.0);
            //Turn
            encoderTurn(0,1,3);
            //Drive Forwards
            distanceEncoderDrive(1,1,1,0, myRobot.frontDistance);
            //Grab the stone
            myRobot.leftClaw.setPosition(1);
            sleep(800);
            //Drive Backwards
            encoderStraightDriveInches(-3,1);
            //Turn
            encoderTurn(-90,1,3);
            //Drive Forwards
            encoderStraightDriveInches(dropDistance + 3*SKYSTONEConstants.doubleAdjustDistance + 3, 1.0);
            //Release the stone
            myRobot.leftClaw.setPosition(.3);
            //myRobot.rightClaw.setPosition(1);
            sleep(800);
            //Drive Backwards
            encoderStraightDriveInches(-10,1);
            break;
        }

    }

    private void crossBridge(SKYSTONEAutonomousMethods methods, SKYSTONEClass myRobot, double speed) {
        Log.d("Skystone Status: ", "Stone Picked Up");
        //methods.encoderStrafeDriveInchesRight(-3, speed);
        //Re-center claw
        //myRobot.clawRotation.setPosition(SKYSTONEConstants.straight);
        //Log.d("Skystone Status: ", "Claw Re-Centered");
        //methods.encoderStraightDriveInches(15, speed);
        //myRobot.runWithEncoder(1, SKYSTONEConstants.raiseTicks-100, myRobot.leftElevator, myRobot.rightElevator);
        //Turn
        myRobot.elevatorDistanceDrive(1, SKYSTONEConstants.raiseTicks+100, 7,2);
        methods.encoderStraightDriveInches(-8, speed);
        myRobot.clawRotation.setPosition(SKYSTONEConstants.straight);
        methods.encoderTurnNoStop(-90, 1, 5);
        Log.d("Skystone Status: ", "Turned");
        //Cross bridge
        double returnDistance;
        if(skyStonePosition.equals("Left")){
            returnDistance = SKYSTONEConstants._bBridgeCrossDistance + SKYSTONEConstants.shiftDistance;
            //methods.encoderStraightDriveInches(SKYSTONEConstants._bBridgeCrossDistance + SKYSTONEConstants.shiftDistance, speed);
        }
        else if(skyStonePosition.equals("Right")){
            returnDistance = SKYSTONEConstants._bBridgeCrossDistance - SKYSTONEConstants.shiftDistance;
            //methods.encoderStraightDriveInches(SKYSTONEConstants._bBridgeCrossDistance - SKYSTONEConstants.shiftDistance, speed);
        }
        else{
            returnDistance = SKYSTONEConstants._bBridgeCrossDistance;
            //methods.encoderStraightDriveInches(SKYSTONEConstants._bBridgeCrossDistance, speed);
        }
        methods.encoderStraightDriveNoStop(returnDistance, 1);
    }
    /*
    private void getSkystonePosition(SKYSTONEVuforiaDetection vuforiaMethods, List<VuforiaTrackable> detections) {
        y = vuforiaMethods.loopDetection(telemetry, detections);
        if(y > SKYSTONEConstants.stoneDiff){
            skyStonePosition = "Right";
            Log.d("SkystonePosition", "Right: " + y);
            telemetry.addData("SkystonePosition", "Right");
        }
        else if(Math.abs(y)< SKYSTONEConstants.stoneDiff){
            skyStonePosition = "Center";
            Log.d("SkystonePosition", "Center: " + y);
        }
        else{
            Log.d("SkyStonePosition", "Left: " + y);
        }
    }
    */
    /*private void dashboardRecordPosition(int deltax, int deltay) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("cat", 3.7);
        packet.fieldOverlay().setFill("blue").fillRect(x,y,x+ deltax,y + deltay +2);

        dashboard.sendTelemetryPacket(packet);
        x = x + deltax;
        y = y + deltay;
    }*/
}
