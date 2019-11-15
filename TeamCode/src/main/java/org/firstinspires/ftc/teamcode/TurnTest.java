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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@Autonomous(name="TurnTest", group="Linear Opmode")
public class TurnTest extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    SKYSTONEClass myRobot = new SKYSTONEClass();
    SKYSTONEAutonomousMethods methods = new SKYSTONEAutonomousMethods() {
        @Override
        public void runOpMode() throws InterruptedException {

        }
    };

    void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        myRobot.initialize(hardwareMap, telemetry);

        this.telemetry = telemetry;
        //Sensors
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        methods.initialize(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Show the elapsed game time and wheel power.
            //encoderTurn(0, 1, 5);
            //methods.frontDistanceEncoderDrive(20,5, 1, -0);
            methods.straighteningEncoderDriveInches(70,0,5,1);
            methods.straighteningEncoderDriveInches(-70, 0, 5, 1);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            break;
        }
    }

    void encoderTurn(double targetAngle, double power, double tolerance){
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(-drivePower, drivePower);
        while(Math.abs(error)>tolerance /*&& opModeisActive()*/) {

            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle - currentAngle);
            drivePower = Math.max(Math.min(error / 90, 1), -1) * Math.abs(power);
            runMotors(-drivePower, drivePower);
            telemetry.addData("Skystone: ", "encoderTurn Error: " + error + " Power: " + drivePower + "CurrentAngle: " + currentAngle);
        }
    }

    void straighteningEncoderDrive(double distance, double targetAngle, double tolerance, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(power, power);
        double curDistance = leftFrontEncoder();
        //double startDistance = curDistance;
        double errorDistance = curDistance-distance;
        double adjust;
        while (Math.abs(errorDistance)>tolerance){
            double currentAngle = getHorizontalAngle();
            double errorAngle = targetAngle-currentAngle;
            double PCoefficient = 1.0/45;
            errorAngle = loopAround(errorAngle);
            double steer = errorAngle*PCoefficient;
            if (distance < 0) {
                steer *= -1.0;
            }
            double leftSpeed = power + steer;
            double rightSpeed = power - steer;
            errorDistance = curDistance-distance;
            runMotors(leftSpeed, rightSpeed);
            curDistance = leftFrontEncoder();
            telemetry.addData("Skystone: ", "DistanceDrive Error: " + errorDistance +
                    " Angle Steer: " + steer + "CurrentDistance: " + curDistance + "Power: " + power);
            Log.d("Skystone: ", "DistanceDrive Error: " + errorDistance +
                    " Angle Steer: " + steer + "CurrentDistance: " + curDistance + "Power: " + power + "Angle" + currentAngle);
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double getHorizontalAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.firstAngle;
        output = loopAround(output);
        return output;
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

    private void setModeAllDrive(DcMotor.RunMode mode){
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }

    void runMotors (double leftPower, double rightPower){
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }

    int leftFrontEncoder(){
        return myRobot.lf.getCurrentPosition();
    }
    int leftBackEncoder(){
        return myRobot.lb.getCurrentPosition();
    }
    int rightFrontEncoder(){
        return myRobot.rf.getCurrentPosition();
    }
    int rightBackEncoder(){
        return myRobot.rb.getCurrentPosition();
    }
}
