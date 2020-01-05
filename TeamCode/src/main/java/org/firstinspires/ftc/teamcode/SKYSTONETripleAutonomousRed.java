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

@Autonomous(name="1TripleAutonomousRed", group="Linear Opmode")
//@Disabled
public class SKYSTONETripleAutonomousRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double dropDistance = SKYSTONEAutonomousConstants.doubleBridgeCross;
    //Second Strafe
    double wallDistance = SKYSTONEAutonomousConstants.doubleWallDistance;
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
        //double wallDistance = 8.0/3 + 8.0;
        initialize(hardwareMap, telemetry);
        //myRobot.frontBase.setPosition(SKYSTONEConstants.flUp);
        //myRobot.frontGrabber.setPosition(SKYSTONEConstants.frUp);
        String skyStonePosition = "Center";
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        getAngleWaitForStart();
        runtime.reset();

        while(opModeIsActive()){
            frontReleaseStone();
            backReleaseStone();
            directionalDrive(SKYSTONEAutonomousConstants.firstStoneDriveX, SKYSTONEAutonomousConstants.firstStoneDriveY, true, 1,0);
            detectSkyStonePosition(SKYSTONEAutonomousConstants.firstStoneDriveY, skyStonePosition);
            encoderStrafeDriveInchesRight(1,1);
            frontGrabStone();
            frontCarryStone();
            directionalDrive(SKYSTONEAutonomousConstants.firstStoneAwayX, SKYSTONEAutonomousConstants.firstStoneAwayY, true, 2,0);
            backCarryStone();
            straighteningEncoderDrive(SKYSTONEAutonomousConstants.firstBridgeCross, 0, 50, 1);
            //grabFoundation();
            directionalDrive(SKYSTONEAutonomousConstants.stoneDropX, SKYSTONEAutonomousConstants.farStoneDropY, true, 2,0);
            frontReleaseStone();
            sleep(250);
            grabFoundation();
            /*
            placeAndReturn(SKYSTONEAutonomousConstants.fieldSize - 27,SKYSTONEAutonomousConstants.fieldSize-6,
                    SKYSTONEAutonomousConstants.fieldSize - 27, 16.0/3+10.0);
            placeAndReturn(SKYSTONEAutonomousConstants.fieldSize - 27, SKYSTONEAutonomousConstants.fieldSize-13,
                    SKYSTONEAutonomousConstants.fieldSize - 27, 8+10.0);
            directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, SKYSTONEAutonomousConstants.fieldSize-21, true, 2,0);
            frontReleaseStone();
            encoderTurn(90, 1, 3);
            encoderTurnNoStopPowers(70, -1,-0.25,3);
            encoderTurnNoStopLeftOnly(0,1,3);
            */
            break;
            }
        AutoTransitioner.transitionOnStop(this, "SKYSTONETeleOp");

    }

    private void grabFoundation() {
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        frontCarryStone();
        encoderStrafeDriveInchesRight(-5,1);
        encoderTurn(90, 1, 3);
        encoderStraightDrive(-7,1);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        sleep(250);
        encoderTurnNoStopPowers(70, -1,-0.5,3, false);
        encoderTurnNoStopLeftOnly(0,1,3);
        encoderStraightDrive(-12,1);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(250);
        encoderStraightDrive(5,1);
    }

    private void detectSkyStonePosition(double baseDistance, String skyStonePosition) {
        if(getHue(myRobot.frontColor)>70){
            skyStonePosition = "Depot";
        }
        else if(getHue(myRobot.backColor)>70){
            skyStonePosition = "Bridge";
        }
        if(skyStonePosition.equals("Center")){
            distanceEncoderDrive(baseDistance+8,0.5,1,0, myRobot.frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        if(skyStonePosition.equals("Bridge")){
            distanceEncoderDrive(baseDistance+16,0.5,1,0, myRobot.frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        Log.d("Skystone: ", " skyStonePosition: " + skyStonePosition);
        telemetry.addData("SkystonePosition", skyStonePosition);
        telemetry.update();
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
