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

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TestBotTeleOp", group="Iterative Opmode")
//@Disabled
public class TestBotTeleOp extends OpMode
{
    float[] leftHsvValues = new float[3];
    float[] rightHsvValues = new float[3];
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SKYSTONEDrivetrainClass myRobot = new SKYSTONEDrivetrainClass();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        myRobot.initializeDriveTrain(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftDrive = -gamepad1.left_stick_y;
        double rightDrive = -gamepad1.right_stick_y;
        myRobot.lf.setPower(leftDrive);
        myRobot.lb.setPower(leftDrive);
        myRobot.rf.setPower(rightDrive);
        myRobot.rb.setPower(rightDrive);
        telemetry.addData("Left Power: ", leftDrive);
        telemetry.addData("Right Power: ", rightDrive);
        telemetry.addData("Left Front: ", myRobot.lf.getCurrentPosition());
        telemetry.addData("Right Front: ", myRobot.rf.getCurrentPosition());
        telemetry.addData("Left Back: ", myRobot.lb.getCurrentPosition());
        telemetry.addData("Right Back: ", myRobot.rb.getCurrentPosition());

        int rightRed = myRobot.rightColor.red();
        int rightBlue = myRobot.rightColor.blue();
        int rightGreen = myRobot.rightColor.green();
        int scale = 255;
        int leftRed = myRobot.leftColor.red();
        int leftBlue = myRobot.leftColor.blue();
        int leftGreen = myRobot.leftColor.green();
        Color.RGBToHSV((int) (leftRed)*scale,
                (int) (leftGreen) * scale,
                (int) (leftBlue) * scale,
                leftHsvValues
                );
        Color.RGBToHSV((int) (rightRed) * scale,
                (int) (rightGreen) * scale,
                (int) (rightBlue) * scale,
                rightHsvValues
        );
        telemetry.addData("LeftColor: ", "Red: " + leftRed + " Blue: " + leftBlue + " Green: " + leftGreen + " Hue: " + leftHsvValues[0]);
        telemetry.addData("RightColor: ", "Red: " + rightRed + " Blue: " + rightBlue + " Green: " + rightGreen + " Hue: " + rightHsvValues[0]);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}

