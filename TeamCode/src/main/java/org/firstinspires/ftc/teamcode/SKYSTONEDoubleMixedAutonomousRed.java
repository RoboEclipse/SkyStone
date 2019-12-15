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

@Autonomous(name="1DoubleMixedAutonomousRed", group="Linear Opmode")
//@Disabled
public class SKYSTONEDoubleMixedAutonomousRed extends SKYSTONEAutonomousMethods {

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
        myRobot.frontLower.setPosition(SKYSTONEConstants.flUp);
        myRobot.frontGrabber.setPosition(SKYSTONEConstants.frUp);
        getAngleWaitForStart();
        runtime.reset();


        skyStonePosition = detectFirstStone(true);
        //Grab the stone
        myRobot.frontLower.setPosition(SKYSTONEConstants.flDown);
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
        turn(-86, 0.6, 0.6, 5);

        encoderStrafeDriveInchesRight(-5, 1);
        //Cross bridge
        adaptiveEncoderDrive(dropDistance + SKYSTONEAutonomousConstants.foundationAlign, -88, 100, 1);
        grabFoundation(speed, false);
        //Turn the foundation
        //Robot turns clockwise, therefore negative power
        turn(78, 1, 0, 5);
        //Drive foundation towards wall
        runMotors(-0.8, -0.8);
        sleep(1000);
        runMotors(0, 0);
        //Release foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(200);
        //Straight to not crash into AFK teammate
        encoderStraightDrive(3, 1);
        //Turn back
        turn(88, 1, 1, 5);
        //Get past skystone so we don't push it
        encoderStrafeDriveInchesRight(8, 1);

        //Drive under bridge
        straighteningEncoderDriveNoStop(SKYSTONEAutonomousConstants.eSkybridge1+SKYSTONEAutonomousConstants.eSkybridge2, 92, 50, 1);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        //Drive forwards
        distanceEncoderDrive(wallDistance,0.3,1,92, myRobot.frontDistance);
        //encoderStraightDrive(-dropDistance - 3*SKYSTONEConstants.doubleAdjustDistance, 1.0);
        //Turn
        turn(0,0.7,0.7, 3);
        encoderStraightDrive(-1,1);
        if(skyStonePosition.equals("Left")){
            //Depot side align to the wall
            encoderStrafeDriveInchesRight(-7,0.5);
        }
        //Drive Forwards
        distanceEncoderDrive(1.9,0.3,1,0, myRobot.frontDistance);

        //Grab the stone
        myRobot.frontLower.setPosition(SKYSTONEConstants.flDown);
        sleep(SKYSTONEAutonomousConstants.frontClawsWaitLength);
        //Drive Backwards
        encoderStraightDrive(-4,1);
        //Turn
        turn(-81, 0.6, 0.6, 5);
        //Drive Forwards
        straighteningEncoderDrive(dropDistance + 3*SKYSTONEAutonomousConstants.doubleAdjustDistance, -81, 50, 1);
        //Release the stone
        myRobot.frontLower.setPosition(SKYSTONEConstants.flUp);
        //myRobot.frontGrabber.setPosition(1);
        sleep(SKYSTONEAutonomousConstants.frontClawsWaitLength);
        //Drive Backwards
        straighteningEncoderDrive(-15, -85, 50, 1);
        AutoTransitioner.transitionOnStop(this, "SKYSTONETeleOp");
    }
}
