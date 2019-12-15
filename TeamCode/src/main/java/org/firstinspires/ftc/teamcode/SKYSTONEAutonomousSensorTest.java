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

@Autonomous(name="SKYSTONEAutonomousSensorTest", group="Linear Opmode")
//@Disabled
public class SKYSTONEAutonomousSensorTest extends SKYSTONEAutonomousMethods {
    private SKYSTONEConstants constants = new SKYSTONEConstants();
    private List<Recognition> updatedRecognitions;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double autoRotate = 0.5;
    private double autoGrab = 0.5;
    private double sideFoundation = 0.5;
    @Override
    public void runOpMode() {

        //SKYSTONEClass methods = new SKYSTONEClass();
        initialize(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("OpModeIsActive", opModeStatus());
            //Drive motor controls
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double speedMultiplier = 1;
            double rotationMultiplier = .8;
            double theta = Math.atan2(lx, ly);
            double v_theta = Math.sqrt(lx * lx + ly * ly);
            double v_rotation = gamepad1.right_stick_x;
            myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);
            autoRotate += 0.06*gamepad2.left_stick_y;
            autoGrab += 0.06*gamepad2.right_stick_y;
            if (gamepad2.dpad_up) {
                sideFoundation += 0.06;
            }
            if (gamepad2.dpad_down) {
               sideFoundation -= 0.06;
            }
            autoRotate = Math.max(Math.min(1, autoRotate),0);
            autoGrab = Math.max(Math.min(1, autoGrab),0);
            sideFoundation = Math.max(Math.min(1, sideFoundation),0);
            myRobot.frontLower.setPosition(autoRotate);
            myRobot.frontGrabber.setPosition(autoGrab);
            myRobot.foundationServo.setPosition(sideFoundation);


            // Show the elapsed game time and wheel power.s
            telemetry.addData("sideFoundaion", sideFoundation);
            telemetry.addData("AutoRotate", autoRotate);
            telemetry.addData("AutoGrab", autoGrab);
            telemetry.addData("HorizontalAngle", getHorizontalAngle());
            telemetry.addData("RollAngle", getRoll());
            telemetry.addData("VerticalAngle", getVerticalAngle());
            telemetry.addData("Encoders: ", "lf: " + leftFrontEncoder() + ", lb: " + leftBackEncoder() +
                    ", rf: " + rightFrontEncoder() + ", rb: " + rightBackEncoder());
            telemetry.addData("BackDistance: ", myRobot.getBackDistance());
            telemetry.addData("FrontDistance: ", myRobot.getFrontDistance());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("LeftStickY", gamepad1.left_stick_y);
            telemetry.addData("RightStickY", gamepad1.right_stick_y);
            telemetry.addData("LeftStickX", gamepad1.left_stick_x);
            telemetry.addData("RightStickX", gamepad1.right_stick_x);
            telemetry.update();
        }
    }

    public boolean opModeCheck(){
        return opModeIsActive();
    }
}
