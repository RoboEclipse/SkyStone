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

@Autonomous(name="SKYSTONESkyStoneAutonomousRed", group="Linear Opmode")
//@Disabled
public class SKYSTONESkyStoneAutonomousRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //double y = 0;
    FtcDashboard dashboard;
    List<Recognition> updatedRecognitions;
    String skyStonePosition = "Left";

    @Override
    public void runOpMode() {

        SKYSTONEAutonomousMethods methods = this;
        SKYSTONEClass myRobot = methods.myRobot;
        SKYSTONEVuforiaDetection vuforiaMethods = new SKYSTONEVuforiaDetection();
        dashboard = FtcDashboard.getInstance();
        final double speed = 1;
        methods.initialize(hardwareMap, telemetry);
        List<VuforiaTrackable> detections = vuforiaMethods.initializeVuforia(hardwareMap);
        vuforiaMethods.activateDetection();
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
            //Put down foundation servos
            myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
            myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
            //Extend Slide Begin
            myRobot.runWithEncoderBegin(0.9, SKYSTONEConstants.extendSlide, myRobot.clawSlide);
            //Move forward first segment
            methods.encoderStraightDriveInches(SKYSTONEConstants._aSkyStoneDistance/4, speed);
            Log.d("Skystone Status:", "First distance traveled");
            //Raise elevator
            myRobot.elevatorDistanceDrive(1, SKYSTONEConstants.raiseTicks+100, 15,2);
            //myRobot.runWithEncoder(1, SKYSTONEConstants.raiseTicks, myRobot.rightElevator, myRobot.leftElevator);
            Log.d("Skystone Status:", "Elevator Raised");
            sleep(800);
            skyStonePosition = myRobot.getSkystonePosition(vuforiaMethods, detections);
            telemetry.addData("Detected: ", skyStonePosition);
            Log.d("Skystone Status: ", "Detected " + skyStonePosition);
            //Move accordingly
            myRobot.runWithEncoderEnd(SKYSTONEConstants.extendSlide, myRobot.clawSlide);
            myRobot.clawSlide.setPower(-0.35);
            methods.backDistanceEncoderDrive(SKYSTONEConstants._pickUpDistance, 0.5, 1);

            if(skyStonePosition.equals("Left")){
                methods.encoderStrafeDriveInchesRight(-SKYSTONEConstants.shiftDistance+SKYSTONEConstants.extraShift, 1);
            }
            else if(skyStonePosition.equals("Right")) {
                methods.encoderStrafeDriveInchesRight(SKYSTONEConstants.shiftDistance + 1.25, 1);
            }
            //Extend slide End
            myRobot.clawSlide.setPower(0);
            Log.d("Skystone Status:", "Slide extended");
            //Continue extend until max length

            //Drive second length and pick up stone
            //TODO: Use Distance Sensor
            //methods.encoderTurnNoStop(0,1,2);


            Log.d("Skystone Status: ", "Second distance traveled");
            methods.pickUpStone();
            sleep(800);
            crossBridge(methods, myRobot, speed);

            Log.d("Skystone Status: ", "Crossed Bridge");
            //Loosen claw and return
            myRobot.clawServo.setPosition(SKYSTONEConstants.loosen);
            Log.d("Skystone Status: ", "Dropped block");
            double wallDistance= SKYSTONEConstants.leftWallDistance;
            wallDistance += 2*SKYSTONEConstants.shiftDistance;
            /*
            if(skyStonePosition.equals("Right")){
                wallDistance += 2*SKYSTONEConstants.shiftDistance;
                //methods.encoderStraightDriveInches(SKYSTONEConstants._bBridgeCrossDistance - SKYSTONEConstants.shiftDistance, speed);
            }
            else if (skyStonePosition.equals("Center")){
                wallDistance += SKYSTONEConstants.shiftDistance-5;
                //methods.encoderStraightDriveInches(SKYSTONEConstants._bBridgeCrossDistance, speed);
            }
            */
            methods.distanceEncoderDrive(wallDistance, 2, 1, -90, myRobot.frontDistance);
            Log.d("Skystone Status: ", "Drive Back for Second Stone");
            methods.encoderTurn(0,1,2);
            myRobot.elevatorDistanceDrive(1, SKYSTONEConstants.raiseTicks+100, 15,2);
            methods.backDistanceEncoderDrive(SKYSTONEConstants._pickUpDistance, 1, 1);
            methods.pickUpStone();
            sleep(800);
            Log.d("Skystone Status: ", "Get Second Stone");
            crossBridge(methods, myRobot, speed);
            myRobot.clawServo.setPosition(SKYSTONEConstants.loosen);
            Log.d("Skystone Status: ", "Deliver Second Stone");
            methods.encoderStraightDriveNoStop(25, 1);


            methods.encoderStraightDriveInches(SKYSTONEConstants._cBridgeReturnDistance + 10, 1);

            Log.d("Skystone Status: ", "Returned");
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Offset", y);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SkyStone", "Location: " + skyStonePosition);
            telemetry.update();
            break;
        }
        vuforiaMethods.deactivateDetection();
        AutoTransitioner.transitionOnStop(this, "SKYSTONETeleOp");

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
