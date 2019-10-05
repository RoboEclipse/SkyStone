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

import android.text.style.UpdateAppearance;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="SKYSTONESkyStoneAutonomous", group="Linear Opmode")
//@Disabled
public class SKYSTONESkyStoneAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int x;
    private int y;
    FtcDashboard dashboard;
    List<Recognition> updatedRecognitions;
    String skyStonePosition = "Center";

    @Override
    public void runOpMode() {

        SKYSTONEAutonomousMethods methods = new SKYSTONEAutonomousMethods() {
            @Override
            public void runOpMode() throws InterruptedException {

            }
        };
        dashboard = FtcDashboard.getInstance();
        final double speed = 1;
        methods.initialize(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            methods.encoderStraightDriveInches(-16, speed);
            methods.encoderStrafeDriveInchesRight(SKYSTONEConstants._aSkyStoneDistance/2, speed);
            updatedRecognitions = methods.runTensorFlow();
            //If anything detected
            if(updatedRecognitions != null){
                //Try and find a Skystone
                for(Recognition recognition : updatedRecognitions){
                    //Get height of image
                    int height = recognition.getImageHeight();
                    //Find skystone
                    if(recognition.getLabel().equals("Skystone")){
                        double center = (recognition.getTop()+recognition.getBottom())/2;
                        if(center>2*height/3){
                            skyStonePosition = "Left";
                        }
                        if(center<height/3){
                            skyStonePosition = "Right";
                        }
                    }
                }
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    Log.i("Label", String.format("label (%d)", i) + recognition.getLabel());
                    Log.i("LeftTopCorner", String.format("  left,top (%d) %.03f , %.03f",
                            i, recognition.getLeft(), recognition.getTop()));
                    Log.i("RightBottomCorner", String.format("  right,bottom (%d) %.03f , %.03f",
                            i, recognition.getRight(), recognition.getBottom()));
                }

            }
            telemetry.addData("SkyStone", "Location: " + skyStonePosition);
            Log.i("SkyStone Location", "Location: " + skyStonePosition);
            telemetry.update();
            //Move accordingly
            if(skyStonePosition.equals("Left")){
                methods.encoderStraightDriveInches(-8, speed);
            }
            else if(skyStonePosition.equals("Right")){
                methods.encoderStraightDriveInches(8, speed);
            }
            methods.encoderStrafeDriveInchesRight(SKYSTONEConstants._aSkyStoneDistance/2, speed);
            //methods.encoderStrafeDriveInchesRight(-3, speed);
            methods.loosenCollector();
            methods.lowerClaw();
            sleep(1000);
            methods.tightenCollector();
            sleep(3000);
            methods.encoderStrafeDriveInchesRight(15, speed);
            methods.encoderStraightDriveInches(SKYSTONEConstants._bBridgeCrossDistance, speed);
            methods.stopCollector();
            methods.higherClaw();
            sleep(1000);
            methods.encoderStraightDriveInches(SKYSTONEConstants._cBridgeReturnDistance, speed);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SkyStone", "Location: " + skyStonePosition);
            telemetry.update();
            break;
        }
        methods.stopTensorFlow();
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
