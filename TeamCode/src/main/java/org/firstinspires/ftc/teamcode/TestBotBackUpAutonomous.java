package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="TestBotBackUpAutonomous", group="Linear Opmode")
//@Disabled
public class TestBotBackUpAutonomous extends SKYSTONEAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private int x;
    // private int y;
    FtcDashboard dashboard;
    double stopTime = 25;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        final double speed = 0.75;
        myRobot.initializeDriveTrain(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        //methods.waitForStart2();
        while (!isStarted()) {
            synchronized (this) {
                try {
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

            while (runtime.seconds() < stopTime) {
                boolean[] stonesStillHere = {true, true, true, true, true, true};
                boolean hasStone = false;

                double founationMovedTime = 25;

                while (myRobot.getBackDistance() < 45) {
                    sleep(50);
                }

                while (!hasStone) {
                    int stoneWeGoingFor = -1;
                    for (int stoneNum = 5; stoneNum >= 0; stoneNum--) {
                        if (stonesStillHere[stoneNum]) {
                            stoneWeGoingFor = stoneNum;
                            break;
                        }
                    }

                    if (stoneWeGoingFor >= 0) {
                        //get along the wall to the right stone position
                        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 3, 3 + 6 * (5 - stoneWeGoingFor), true, 2, 0);

                            /*
                            //need to install distance sensor on top of left side of robot
                            while(myRobot.getLeftUpperDistance()<45){
                                sleep(50);
                            }
                            */

                            /*
                            //need to install distance sensor on the bottom of the left side of the robot
                            if (myRobot.getLeftBottomDistance()<72){
                                // go to stone
                                directionalDriveWithInterrupt(SKYSTONEAutonomousConstants.fieldSize - 27 , 3 + 6*(5-stoneNum), true, 2,0, myRobot.frontDistance);
                                grabStone()
                                hasStone=True;
                            }
                            else{
                                stonesStillHere[stoneWeGoingFor]=false;
                                continue;
                            }
                            */
                    } else {
                        //park
                    }

                }
                while (hasStone) {

                    directionalDriveWithInterrupt(SKYSTONEAutonomousConstants.fieldSize - 3, 8.0 / 3 + 15.0, true, 2, 0, myRobot.frontDistance);
                    directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 3, 8.0 / 3 + 15.0, true, 2, 0);

                    straighteningEncoderDrive(-65, 0, 50, 1);

                    if (runtime.seconds() < founationMovedTime) {
                        //go to foundation's original position
                        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 3, SKYSTONEAutonomousConstants.fieldSize - 3, true, 2, 0);
                        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, SKYSTONEAutonomousConstants.fieldSize - 3, true, 2, 0);
                        //placeStone();
                        hasStone = false;
                    } else {
                        // go to edge of turned foundation
                        directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 3, SKYSTONEAutonomousConstants.fieldSize - 27, true, 2, 0);
                        encoderTurn(90, 1, 3);
                        //placeStone();
                        encoderTurn(0, 1, 3);
                        hasStone = false;
                    }


                }

            }

            //park
            //break opmode
            break;
        }


    }




    void directionalDriveWithInterrupt(double targetX, double targetY, boolean PID, double tolerance, double targetAngle, DistanceSensor sensor){
        double xDis = targetX - myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
        double yDis = targetY - myRobot.backDistance.getDistance(DistanceUnit.INCH);
        double totalDistance;
        double kR = -0.01;
        double maxVelocity = 1;
        double velocity = maxVelocity;
        double rotationVelocity = 0;
        double kP = 1.0/20;
        double currentAngle;
        double currentError;
        double xRaw = 0;
        double yRaw = 0;
        Localizer.Corner corner;

        if(targetX<SKYSTONEAutonomousConstants.fieldSize/2){
            if(targetY<SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Localizer.Corner.LEFT_DOWN;
            }
            else{
                corner = Localizer.Corner.LEFT_UP;
            }
        } else {
            if(targetY<SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Localizer.Corner.RIGHT_DOWN;
            }
            else{
                corner = Localizer.Corner.RIGHT_UP;
            }
        }
        Log.d("Skystone: ", "kP " + kP + " kR " + kR + " tolerance" + tolerance + " backDistance: " + (targetY-yDis) + " rightDistance: " + (targetX-xDis));
        while((Math.abs(yDis)>tolerance || Math.abs(xDis)>tolerance) && opModeIsActive()){

            switch (corner){
                case LEFT_DOWN:
                    xRaw = myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
                    yRaw = myRobot.backDistance.getDistance(DistanceUnit.INCH);

                    break;
                case LEFT_UP:
                    xRaw = myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
                    yRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                    break;
                case RIGHT_DOWN:
                    xRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
                    yRaw = myRobot.frontDistance.getDistance(DistanceUnit.INCH);

                    break;
                case RIGHT_UP:
                    xRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
                    yRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.backDistance.getDistance(DistanceUnit.INCH);
                    break;
            }

            if (sensor.getDistance(DistanceUnit.INCH)<20){
                if (corner == Localizer.Corner.LEFT_DOWN || corner == Localizer.Corner.LEFT_UP){
                    directionalDrive(0 + 2, yRaw, true, 3, 0);
                }
                else {
                    directionalDrive(144 - 2, yRaw, true, 3, 0);
                }
                break;
            }

            xDis = targetX - xRaw;
            yDis = targetY - yRaw;
            if(xRaw>250 || yRaw>250){
                Log.d("Skystone: ", "FinalX: "  + xRaw + " FinalY: " + yRaw);
                break;
            }
            currentAngle = loopAround(getHorizontalAngle());
            currentError = targetAngle - currentAngle;
            rotationVelocity = currentError * kR;
            telemetry.addData("Skystone: xDis = " + xDis + " yDis = " + yDis,"");
            Log.d("Skystone:", "GyroError: " + currentError + " Rotation Velocity: " + rotationVelocity);
            Log.d("Skystone:"," Targets: targetX = " + targetX + " targetY = " + targetY);
            Log.d("Skystone: ", "xRaw: " + xRaw + " yRaw: " + yRaw);
            totalDistance = Math.sqrt(xDis*xDis+yDis*yDis);
            if(PID) {
                velocity = Math.max(0.25, Math.abs(maxVelocity * getP(totalDistance, kP)));
            }
            double angle = Math.atan2(xDis,yDis);
            if(corner == Localizer.Corner.RIGHT_UP || corner == Localizer.Corner.RIGHT_DOWN){
                angle = Math.PI+angle;
            }
            freeDrive(angle, velocity, rotationVelocity);
            Log.d("Skystone: ", "Skystone Angle: "+ (angle*180/Math.PI) + "Velocity: " + velocity+ " RotationVelocity" + rotationVelocity);

        }
    }
    private double loopAround(double output) {
        if (output > 180) {
            output -= 360;
        }
        if (output < -180) {
            output += 360;
        }
        return output;
    }


}

