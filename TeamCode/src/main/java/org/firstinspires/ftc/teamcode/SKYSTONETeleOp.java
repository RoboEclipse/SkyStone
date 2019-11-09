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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="SKYSTONETeleOp", group="Iterative Opmode")
//@Disabled
public class SKYSTONETeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SKYSTONEClass myRobot = new SKYSTONEClass();
    private double clawRotator = SKYSTONEConstants.straight;
    private double clawPosition = 0;
    private double leftFoundationPosition = SKYSTONEConstants.lDown;
    private double rightFoundationPosition = SKYSTONEConstants.rDown;
    private double collectorPower = 0;
    private double frontClawPosition = 1.0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
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
        //Drive motor controls
        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = .8;
        if(gamepad1.dpad_up){
            ly=-1;
            speedMultiplier = 0.5;
        }
        else if(gamepad1.dpad_down){
            ly=1;
            speedMultiplier = 0.5;
        }
        if(gamepad1.dpad_left){
            lx=1;
            speedMultiplier = 0.5;
        }
        else if(gamepad1.dpad_right){
            lx=-1;
            speedMultiplier = 0.5;
        }
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = -gamepad1.right_stick_x;

        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);

        //Elevator controls
        double elevatorPower = -gamepad2.left_stick_y;
        myRobot.runElevatorMotors(elevatorPower);

        //Slide controls
        double slidePower = gamepad2.right_stick_y;
        myRobot.clawSlide.setPower(slidePower);
        if ((slidePower > 0.001) || (slidePower < -0.001)){
            if (clawPosition != SKYSTONEConstants.tighten){
                Log.d("Protect Claw", "Claw was loose when moving so it got closed");
            }
            clawPosition = SKYSTONEConstants.tighten;
        }

        //Claw rotation
        int horizSlidePosition = myRobot.clawSlide.getCurrentPosition();
        if(gamepad2.dpad_left){
            clawRotator = SKYSTONEConstants.right90;
        }
        else if(gamepad2.dpad_right) {
            clawRotator = SKYSTONEConstants.left90;
        }
        else if(gamepad2.dpad_down){
            clawRotator = SKYSTONEConstants.straight;
        }
        else if(gamepad2.dpad_up){
            clawRotator = SKYSTONEConstants.oppositeSide;
        }

        /* if (clawRotator != SKYSTONEConstants.straight && (horizSlidePosition > SKYSTONEConstants.safeSlide)){
            clawRotator = SKYSTONEConstants.straight;
            Log.d("Protected Claw Rotation", "Rotation Servo set straight because it was not in safe distance");
            telemetry.addData("Protected Claw Rotation", "Rotation Servo Set Straight");
        }*/

        //Claw controls
        if(gamepad2.y) {
            clawPosition = SKYSTONEConstants.tighten;
        }
        else if(gamepad2.x) {
            clawPosition = SKYSTONEConstants.loosen;
        }

        //Collector Servos
        if(gamepad2.right_trigger>0.7){
            collectorPower = 0.75;
        }
        else if(gamepad2.left_trigger>0.7){
            collectorPower = -0.75;
        }
        else{
            collectorPower = 0;
        }

        //Side Claw test{
        if(gamepad2.right_bumper){
            frontClawPosition = 1;
        }
        if(gamepad2.left_bumper){
            frontClawPosition = 0.5;
        }
        myRobot.moveFrontClaw(frontClawPosition         );

        //Foundation Servo Control (testing)
        if(gamepad1.left_trigger>0.7){
            leftFoundationPosition = SKYSTONEConstants.lUp;
            rightFoundationPosition = SKYSTONEConstants.rUp;
        }
        if(gamepad1.right_trigger>0.7){
            leftFoundationPosition = SKYSTONEConstants.lDown;
            rightFoundationPosition = SKYSTONEConstants.rDown;

        }

        //Reset to Autonomous starting position
        /* if(gamepad1.x){
            myRobot.resetAutonomous();
            clawRotator = SKYSTONEConstants.straight;
        }*/


        //Autonomous Tests
        if(gamepad2.a){
            pickUpStone();
        }
        myRobot.rotateStackingClaw(clawRotator);
        myRobot.grabStones(clawPosition);
        myRobot.runCollectorServos(collectorPower);
        myRobot.leftFoundationServo.setPosition(leftFoundationPosition);
        myRobot.rightFoundationServo.setPosition(rightFoundationPosition);
        // Show the elapsed game time and wheel power.
        telemetry.addData("ElevatorPower", elevatorPower);
        telemetry.addData("SlidePower", slidePower);
        telemetry.addData("ClawRotationPosition", clawRotator);
        telemetry.addData("ClawPosition", clawPosition);
        telemetry.addData("leftFoundationPosition", leftFoundationPosition);
        telemetry.addData("CollectorPower", collectorPower);
        //telemetry.addData("ElevatorDistance", myRobot.getElevatorDistance());
        //telemetry.addData("BackDistance", myRobot.getBackDistance());
        myRobot.readEncoders();

        //Log
        Log.d("ElevatorPower", String.valueOf(elevatorPower));
        Log.d("SlidePower", String.valueOf(slidePower));
        Log.d("ClawRotationPosition", String.valueOf(clawRotator));
        Log.d("ClawPosition", String.valueOf(clawPosition));
        Log.d("LeftFoundationPosition", String.valueOf(leftFoundationPosition));

        Log.d("CollectorPower", String.valueOf(collectorPower));
        Log.d("Encoders",
        "lf: " + myRobot.lf.getCurrentPosition()
            + " lb: " + myRobot.lb.getCurrentPosition()
            + " rf: " + myRobot.rf.getCurrentPosition()
            + " rb: "+ myRobot.rb.getCurrentPosition()
            + " left elevator: " + myRobot.leftElevator.getCurrentPosition()
            + " right elevator: " + myRobot.rightElevator.getCurrentPosition()
            + " slide motor: " + horizSlidePosition
        );

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    private int stage = 0;
    private ElapsedTime timer = new ElapsedTime();
    private void pickUpStone(){
        //State 1: Raise elevator, then rotate claw
        if(stage == 0 && timer.milliseconds()>=2000){
            myRobot.runWithEncoder(0.5, 400, myRobot.rightElevator, myRobot.leftElevator);
            clawRotator = SKYSTONEConstants.left90;
            timer.reset();
            stage++;
        }
        //State 2: Lower elevator, then grab stone
        if(stage == 1 && timer.milliseconds()>=2000){
            myRobot.runWithEncoder(0.5, -400, myRobot.rightElevator, myRobot.leftElevator);
            clawPosition = SKYSTONEConstants.tighten;
            timer.reset();
            stage++;
        }
        //State 3: Straighten Claw
        if(stage == 2 && timer.milliseconds()>=2000){
            clawRotator = SKYSTONEConstants.straight;
            timer.reset();
            stage=0;
        }
    }

    void autoFoundationPull(){

    };
}

