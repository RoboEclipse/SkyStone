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

@Autonomous(name="SKYSTONEMixedAutonomousBlue", group="Linear Opmode")
//@Disabled
public class SKYSTONEMixedAutonomousBlue extends SKYSTONEAutonomousMethods {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //double y = 0;
    FtcDashboard dashboard;
    List<Recognition> updatedRecognitions;

    private double dropDistance = SKYSTONEAutonomousConstants.doubleBridgeCross;

    @Override
    public void runOpMode() {
        String skyStonePosition;
        dashboard = FtcDashboard.getInstance();
        final double speed = 1;
        final double foundationGrabAngle = 178;
        initialize(hardwareMap, telemetry);
        myRobot.leftClaw.setPosition(SKYSTONEConstants.flUp);
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frUp);
        getAngleWaitForStart();
        runtime.reset();


        skyStonePosition = detectFirstStone(false);
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frDown);
        sleep(800);
        //Drive backwards
        encoderStraightDriveNoStop(-4, 1);
        if(skyStonePosition.equals("Left")){
            dropDistance-=SKYSTONEAutonomousConstants.doubleAdjustDistance;
        }
        else if(skyStonePosition.equals("Right")){
            dropDistance += SKYSTONEAutonomousConstants.doubleAdjustDistance;
        }
        encoderTurn(88, 1.0, 1);
        //Cross bridge
        encoderStraightDriveInches(dropDistance, 1);
        grabFoundation(speed, foundationGrabAngle);
        //Turn the foundation
        //Robot turns clockwise, therefore negative power
        encoderTurnNoStopRightOnly(-82, 1, 3);
        //Drive foundation towards wall
        runMotors(-1, -1);
        sleep(1000);
        runMotors(0, 0);
        //Strafe to make sure foundation goes into building zone
        encoderStrafeDriveInchesRight(20, speed);
        //Release foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(200);
        encoderTurn(-88, 1,3);
        //Let go of stone
        myRobot.rightClaw.setPosition(SKYSTONEConstants.frUp);
        //Get past skystone so we don't push it
        encoderStrafeDriveInchesRight(-SKYSTONEAutonomousConstants.skystoneClear-4,1);
        //Drive under bridge
        encoderStraightDriveInches(SKYSTONEAutonomousConstants.eSkybridge1+10, 0.6);
        AutoTransitioner.transitionOnStop(this, "SKYSTONETeleOp");
    }
}
