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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

@Autonomous(name="TestBotDoubleAutonomousRed", group="Linear Opmode")
//@Disabled
public class TestBotDoubleAutonomousRed extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor frontSensor;
    private DistanceSensor backSensor;
    // private int x;
    // private int y;
    double speed = 1;
    String skysStonePosition = "Center";
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        SKYSTONEDrivetrainClass drivetrain = myRobot;
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontSensor = hardwareMap.get(DistanceSensor.class, "frontDistance");
        backSensor = hardwareMap.get(DistanceSensor.class, "backDistance");
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        while (!isStarted()) {
            synchronized (this) {
                try {
                    //telemetry.addData("Distance", myRobot.getBackDistance() + "");
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
            //Drive the rest of the distance
            distanceEncoderDrive(1,1,1,0, frontSensor);
            float leftHue = hsv(myRobot.leftColor);
            float rightHue = hsv(myRobot.rightColor);
            if(leftHue >= 115) {
                skysStonePosition = "Left";
            }
            else if(rightHue >= 115) {
                skysStonePosition  = "Right";
            }
            else {
                skysStonePosition = "Center";
            }
            if(skysStonePosition.equals("Left")) {
                encoderStrafeDriveInchesRight(-SKYSTONEAutonomousConstants.doubleAdjustDistance,1);
            }
            if(skysStonePosition.equals("Right")) {
                encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.doubleAdjustDistance, 1);
            }
            //Drive backwards
            encoderStraightDriveInches(-3, 1);
            //Turn
            encoderTurn(-90, 1.0, 2);
            //Drive forwards
            double dropDistance = SKYSTONEAutonomousConstants.doubleBridgeCross+SKYSTONEAutonomousConstants.doubleAdjustDistance;
            double wallDistance = SKYSTONEAutonomousConstants.doubleWallDistance-SKYSTONEAutonomousConstants.doubleAdjustDistance;
            if(skysStonePosition.equals("Left")) {
                wallDistance += (SKYSTONEAutonomousConstants.doubleAdjustDistance);
                dropDistance += (SKYSTONEAutonomousConstants.doubleAdjustDistance);
            }
            if(skysStonePosition.equals("Right")) {
                wallDistance -= (SKYSTONEAutonomousConstants.doubleAdjustDistance);
                dropDistance -= SKYSTONEAutonomousConstants.doubleAdjustDistance;
            }
            encoderStraightDriveInches(dropDistance, 1);
            //Drive backwards
            distanceEncoderDrive(wallDistance,1,-1,-90, backSensor);
            //encoderStraightDriveInches(-dropDistance - 3*SKYSTONEConstants.doubleAdjustDistance, 1.0);
            //Turn
            encoderTurn(0,1,2);
            //Drive Forwards
            distanceEncoderDrive(1,1,1,0, frontSensor);
            //Drive Backwards
            encoderStraightDriveInches(-3,1);
            //Turn
            encoderTurn(-90,1,2);
            //Drive Forwards
            encoderStraightDriveInches(dropDistance + 3*SKYSTONEAutonomousConstants.doubleAdjustDistance + 3, 1.0);
            //Drive Backwards
            encoderStraightDriveInches(-15,1);
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
