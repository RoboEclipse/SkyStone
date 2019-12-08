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

@Autonomous(name="1DoubleMixedAutonomousBlue", group="Linear Opmode")
//@Disabled
public class SKYSTONEDoubleMixedAutonomousBlue extends SKYSTONEAutonomousMethods {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double wallDistance = SKYSTONEAutonomousConstants.doubleWallDistance - 1;
    //double y = 0;
    FtcDashboard dashboard;
    List<Recognition> updatedRecognitions;

    private double dropDistance = SKYSTONEAutonomousConstants.doubleBridgeCross;

    @Override
    public void runOpMode() {
        String skyStonePosition;
        dashboard = FtcDashboard.getInstance();
        final double speed = 1;
        initialize(hardwareMap, telemetry);
        myRobot.leftClaw.setPosition(SKYSTONEConstants.flUp);
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frUp);
        getAngleWaitForStart();
        runtime.reset();
        skyStonePosition = detectFirstStone(false);
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frDown);
        sleep(SKYSTONEAutonomousConstants.frontClawsWaitLength);
        //Drive backwards
        encoderStraightDriveNoStop(-5, 1);
        if(skyStonePosition.equals("Left")){
            dropDistance+=SKYSTONEAutonomousConstants.doubleAdjustDistance;
            wallDistance = 3;
        }
        else if(skyStonePosition.equals("Right")){
            dropDistance -= SKYSTONEAutonomousConstants.doubleAdjustDistance;
            wallDistance = 15;
        }
        turn(90, 0.8, 0.8, 5);
        //Cross bridge
        encoderStrafeDriveInchesRight(-5, 1);
        adaptiveEncoderDrive(dropDistance + SKYSTONEAutonomousConstants.foundationAlign, 90, 100, 1);
        grabFoundation(speed, true);
        //Turn the foundation
        //Robot turns clockwise, therefore negative power
        turn(-74, 0, 1, 5);
        //Drive foundation towards wall
        runMotors(-0.8, -0.8);
        sleep(1000);
        runMotors(0, 0);
        //Release foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(200);
        //Strafe to not crash into AFK teammate
        encoderStraightDriveInches(3, 1);
        //Turn back
        turn(-90, 1, 1, 5);
        //Get past skystone so we don't push it
        // -5 will crash into brig
        encoderStrafeDriveInchesRight(-3 ,1);
        //Drive under bridge
        straighteningEncoderDriveInches(SKYSTONEAutonomousConstants.eSkybridge1 + SKYSTONEAutonomousConstants.eSkybridge2, -90, 50, 1);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        //Drive backwards
        distanceEncoderDrive(wallDistance,0.3, 1, -90, myRobot.frontDistance);
        //encoderStraightDriveInches(-dropDistance - 3*SKYSTONEConstants.doubleAdjustDistance, 1.0);
        //Turn
        turn(0,0.8,0.8, 5);
        encoderStraightDriveInches(-1,1);
        if(skyStonePosition.equals("Left")){
            //Depot side align to the wall
            encoderStrafeDriveInchesRight(6,0.5);
        }
        //Drive Forwards
        distanceEncoderDrive(1.9,0.3,1,0, myRobot.frontDistance);

        //Grab the stone
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frDown);
        sleep(SKYSTONEAutonomousConstants.frontClawsWaitLength);
        //Drive Backwards
        encoderStraightDriveInches(-4,1);
        //Turn
        turn(90,0.6, 0.6, 5);
        //Drive Forwards
        adaptiveEncoderDrive(dropDistance + 3*SKYSTONEAutonomousConstants.doubleAdjustDistance, 90, 50, 1.0);
        //Release the stone
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frUp);
        //myRobot.rightClaw.setPosition(1);
        sleep(SKYSTONEAutonomousConstants.frontClawsWaitLength);
        //Drive Backwards
        encoderStraightDriveInches(-20,1);
        AutoTransitioner.transitionOnStop(this, "SKYSTONETeleOp");
    }
}
