package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

abstract class SKYSTONEAutonomousMethods extends LinearOpMode {
    //Hardware
    // The IMU sensor object
    //BNO055IMU imu;
    Orientation angles;
    //Software
    //private Telemetry telemetry;

    int turnPValue = 15;
    //Classes

    private SKYSTONEConfiguration skystoneNames = new SKYSTONEConfiguration();
    private SKYSTONEConstants skystoneConstants = new SKYSTONEConstants();
    SKYSTONEClass myRobot = new SKYSTONEClass();
    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        myRobot.initialize(hardwareMap, telemetry);
        
        this.telemetry = telemetry;
        //Sensors
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.


        //Tensorflow
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        /*
        initVuforia(hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        } else {
            Log.d("Sorry!", "This device is not compatible with TFOD");
        }
*/
        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
    }

    //Methods
    //TODO: Incorporate freeEncoderDrive, encoderDrive and runUsingEncoder
    void freeEncoderDrive(int lf, int rf, int lb, int rb, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElapsedTime time = new ElapsedTime();
        myRobot.lf.setTargetPosition(lf*(int)SKYSTONEConstants.TICKS_PER_INCH);
        myRobot.rf.setTargetPosition(rf*(int)SKYSTONEConstants.TICKS_PER_INCH);
        myRobot.lb.setTargetPosition(lb*(int)SKYSTONEConstants.TICKS_PER_INCH);
        myRobot.rb.setTargetPosition(rb*(int)SKYSTONEConstants.TICKS_PER_INCH);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && time.milliseconds()<4000 && opModeIsActive()){
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void encoderStraightDrive(double inches, double power){
        encoderStraightDriveNoStop(inches, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderStraightDriveNoStop(double inches, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderStraightDriveNoReset(inches, power);
    }

    void encoderStraightDriveNoReset(double inches, double power){
        ElapsedTime time = new ElapsedTime();
        multiSetTargetPosition(inches* SKYSTONEConstants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && time.milliseconds()<4000 && opModeIsActive()){
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
    }

    //Negative = Left, Positive = Right
    void encoderStrafeDriveInchesRight(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.lf.setTargetPosition((int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.lb.setTargetPosition(-(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.rf.setTargetPosition(-(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        myRobot.rb.setTargetPosition((int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.lb, myRobot.rf, myRobot.rb) && opModeIsActive()){
            Log.d("SkyStone Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("SkyStone Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("SkyStone Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("SkyStone Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void newEncoderStrafeDriveInchesRight(double inches){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int lfTarget = (int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH);
        int lbTarget = -(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH);
        int rfTarget = lbTarget;
        int rbTarget = lfTarget;
        boolean reached = false;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(1, 1);
        while (!reached && opModeIsActive()){
            reached = true;
            int lfPosition = myRobot.lf.getCurrentPosition();
            int lbPosition = myRobot.lb.getCurrentPosition();
            int rfPosition = myRobot.rf.getCurrentPosition();
            int rbPosition = myRobot.rb.getCurrentPosition();
            int lfError = lfTarget-lfPosition;
            int lbError = lbTarget-lbPosition;
            int rfError = rfTarget-rfPosition;
            int rbError = rbTarget-rbPosition;

            if(Math.abs(lfError)>20){
                myRobot.lf.setPower(Math.max(Math.abs(lfError/500), 0.1)*Math.signum(lfError));
            }
            else{
                myRobot.lf.setPower(0);
                reached = false;
            }
            if(Math.abs(lbError)>20){
                myRobot.lb.setPower(Math.max(Math.abs(lbError/500), 0.1)*Math.signum(lbError));
            }
            else{
                myRobot.lb.setPower(0);
                reached = false;
            }
            if(Math.abs(rfError)>20){
                myRobot.rf.setPower(Math.max(Math.abs(rfError/500), 0.1)*Math.signum(rfError));
            }
            else{
                myRobot.rf.setPower(0);
                reached = false;
            }
            if(Math.abs(rbError)>20){
                myRobot.rb.setPower(Math.max(Math.abs(rbError/500), 0.1)*Math.signum(rbError));
            }
            else{
                myRobot.rf.setPower(0);
                reached = false;
            }
            Log.d("SkyStone Left Front: ", lfPosition+"");
            Log.d("SkyStone Left Back: ", lbPosition+"");
            Log.d("SkyStone Right Front: ", rfPosition+"");
            Log.d("SkyStone Right Back: ", rbPosition+"");
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Positive = Clockwise, Negative = Counterclockwise
    void encoderTurn(double targetAngle, double power, double tolerance){
        encoderTurnNoStop(targetAngle, power, tolerance);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderTurnNoStop(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, -power, power, tolerance, true);
    }
    void encoderTurnNoStopPowers(double targetAngle, double leftPower, double rightPower, double tolerance, boolean usePID) {
        double kR = SKYSTONEAutonomousConstants.kR;
        double kD = SKYSTONEAutonomousConstants.kD;

        //Undefined constants
        double d;
        double dt;
        double leftProportionalPower = leftPower;
        double rightProportionalPower = rightPower;
        //Initial error
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double previousError = error;
        //Initial Time
        ElapsedTime clock = new ElapsedTime();
        double t1 = clock.nanoseconds();
        double t2 = t1;
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive()){

                //Getting Error
                currentAngle = getHorizontalAngle();
                error = loopAround(targetAngle-currentAngle);
            if(usePID){
                //Getting time difference
                t2 = clock.nanoseconds();
                dt = t2-t1;

                //Setting d action
                d = (error-previousError)/dt*Math.pow(10,9);
                //Setting p action
                leftProportionalPower = Math.max(Math.min(error*kR + d*kD, 1),-1)*leftPower;
                rightProportionalPower = Math.max(Math.min(error*kR + d*kD, 1),-1)*rightPower;
                Log.d("Skystone: ", "leftProportionalPower: " + leftProportionalPower + " rightProportionalPower: " + rightProportionalPower);
                Log.d("Skystone: ", "dt: " + dt + "DerivativeAction: " + d*kD);
            }
            else{
                leftProportionalPower = leftPower*Math.signum(error);
                rightProportionalPower = rightPower*Math.signum(error);
            }

            //Set real power
            double realLeftPower = Math.max(Math.abs(leftPower/2), Math.abs(leftProportionalPower))*Math.signum(leftProportionalPower);
            double realRightPower = Math.max(Math.abs(rightPower/2), Math.abs(rightProportionalPower))*Math.signum(rightProportionalPower);
            runMotors(realLeftPower, realRightPower);

            //Store old values
            previousError = error;
            if(usePID){
                t1 = t2;
            }


            //Logging
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + realLeftPower + "rightPower: " + realRightPower + "CurrentAngle: " + currentAngle);
        }
    }

    void turn(double targetAngle, double leftPower, double rightPower, double tolerance){
        double kR = 1/60;
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double leftDrivePower;
        double rightDrivePower;
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive()){
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle-currentAngle);
            leftDrivePower = Math.max(Math.min(error*kR, 1),-1)*Math.abs(leftPower);
            rightDrivePower = Math.max(Math.min(error*kR, 1),-1)*Math.abs(rightPower);
            leftDrivePower = floorPower(leftDrivePower, 0.45);
            rightDrivePower = floorPower(rightDrivePower, 0.45);
            runMotors(-leftDrivePower, rightDrivePower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + leftDrivePower + "rightPower: " + rightDrivePower + "CurrentAngle: " + currentAngle);
        }
    }

    void encoderTurnNoStopLeftOnly(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, -power, 0, tolerance, true);
    }

    void encoderTurnNoStopRightOnly(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, 0, power, tolerance, true);
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
    void straighteningEncoderDrive(double inches, double targetAngle, double tolerance, double power){
        straighteningEncoderDriveNoStop(inches, targetAngle, tolerance, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void straighteningEncoderDriveNoStop(double inches, double targetAngle, double tolerance, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        straighteningEncoderDriveNoReset(inches, targetAngle, tolerance, power);
    }

    void straighteningEncoderDriveNoReset(double inches, double targetAngle, double tolerance, double power){
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runMotors(power, power);
        double curDistance = leftFrontEncoder();
        double targetDistance = inches*SKYSTONEConstants.TICKS_PER_INCH;
        //double startDistance = curDistance;
        double errorDistance = targetDistance-curDistance;
        double adjust;
        while (Math.abs(errorDistance)>tolerance && opModeIsActive()){
            double currentAngle = getHorizontalAngle();
            double steer = getCorrection(currentAngle, targetAngle);
            adjust = Math.max(Math.min(errorDistance, 40),-40)/40*power;
            double leftSpeed = adjust - steer;
            double rightSpeed = adjust + steer;
            errorDistance = targetDistance-curDistance;
            runMotors(leftSpeed, rightSpeed);
            curDistance = leftFrontEncoder();
            Log.d("Skystone: ", "DistanceDrive Error: " + errorDistance +
                    " Angle Steer: " + steer + "CurrentDistance: " + curDistance + "Power: " + adjust + "Angle" + currentAngle);
        }
    }


    void adaptiveEncoderDrive(double inches, double targetAngle, double tolerance, double power){
        double fastMode;
        if(Math.abs(inches)>30){
            fastMode = (Math.abs(inches)-15)*Math.signum(inches);
            straighteningEncoderDriveNoStop(fastMode, targetAngle, tolerance, power);
        }
        encoderStraightDriveNoReset(inches, power);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getCorrection(double currentAngle, double targetAngle){
        double errorAngle = loopAround(targetAngle-currentAngle);
        double PCoefficient = 1.0/turnPValue;
        return errorAngle*PCoefficient;
    }

    void distanceEncoderDrive(double distance, double tolerance, double power, double targetAngle, DistanceSensor sensor){
        distanceEncoderDriveNoStop(distance, tolerance, power, targetAngle, sensor);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void distanceEncoderDriveNoStop(double distance, double tolerance, double power, double targetAngle, DistanceSensor sensor) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runMotors(power, power);
        double curDistance = sensor.getDistance(DistanceUnit.INCH);
        //double startDistance = curDistance;
        double error = curDistance-distance;
        double adjust;
        while (Math.abs(error)>tolerance && opModeIsActive()){
            /*
            adjust = 0.1*error/Math.abs(error) + 0.9*(Math.min(Math.abs(error),10))/10*power;
            if(error*power<0){
                adjust*=-1;
            }
             */
            double curAngle = getHorizontalAngle();
            //Get correction
            double steer = getCorrection(curAngle, targetAngle);
            //Get error
            error = curDistance-distance;
            //Once below 20 inches away, start slowing
            adjust = Math.max(Math.min(error, SKYSTONEAutonomousConstants.reducePowerDistance),-SKYSTONEAutonomousConstants.reducePowerDistance)/SKYSTONEAutonomousConstants.reducePowerDistance*power;
            //Ensure power is above 0.2
            adjust = floorPower(adjust);
            runMotors(adjust - steer, adjust + steer);
            curDistance = sensor.getDistance(DistanceUnit.INCH);
            Log.d("Skystone: ", "FrontDistanceDrive Error: " + error + " Adjust: " + adjust + "CurrentDistance: " + curDistance + "Steer: " + steer);
        }
    }

    private double floorPower(double power) {
        if(power > 0){
            power = Math.max(power, SKYSTONEAutonomousConstants.flooringPower);
        }
        else{
            power = Math.min(power, -SKYSTONEAutonomousConstants.flooringPower);
        }
        return power;
    }
    private double floorPower(double power, double floor) {
        if (Math.abs(power) < 0.00001){
            return power;
        }
        if(power > 0){
            power = Math.max(power, floor);
        }
        else{
            power = Math.min(power, -floor);
        }
        return power;
    }
    //Attachments


    //private void
    //Shortcuts
    void setModeAllDrive(DcMotor.RunMode mode){
        myRobot.lb.setMode(mode);
        myRobot.lf.setMode(mode);
        myRobot.rb.setMode(mode);
        myRobot.rf.setMode(mode);
    }
    private void multiSetTargetPosition(double ticks, DcMotor...motors){
        for(DcMotor motor:motors){
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }
    private boolean anyBusy(){
        return myRobot.lb.isBusy() || myRobot.lf.isBusy() || myRobot.rb.isBusy() || myRobot.rf.isBusy();
    }
    private boolean notCloseEnough(int tolerance, DcMotor...motors){
        for(DcMotor motor : motors){
            if(Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) > tolerance){
                return true;                
            }
        }
        return false;
    }
    void runMotors (double leftPower, double rightPower){
        myRobot.lb.setPower(leftPower);
        myRobot.lf.setPower(leftPower);
        myRobot.rb.setPower(rightPower);
        myRobot.rf.setPower(rightPower);
    }
    //IMU Stuff
    double getHorizontalAngle(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    double getRoll(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        output = loopAround(output);
        return output;
    }
    double getVerticalAngle(){
        angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        output = loopAround(output);
        return output;
    }
    boolean opModeStatus(){
        return opModeIsActive();
    }
    float getHue(ColorSensor sensor){
        float[] values = new float[3];
        int scale = 255;
        Color.RGBToHSV(sensor.red()*scale,
                sensor.green() * scale,
                sensor.blue() * scale, values
        );
        return values[0];
    }
    //For carrying the blocks
    void backCarryStone () {
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
    }
    //Basic grab
    void backGrabStone (){
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbDown);
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
        sleep(300);
    }
    //frontRelease is both the release and the starting position before grab
    void backReleaseStone(){
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbReady);
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsReady);
    }
    void frontCarryStone () {
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbUp);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
    }
    //Basic grab
    void frontGrabStone (){
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbDown);
        sleep(100);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
        sleep(300);
    }
    //frontRelease is both the release and the starting position before grab
    void frontReleaseStone(){
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbReady);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsReady);
    }

    //VuforiaDetectionStuff

    //Vuforia Stuff

    //Complex Methods
    void pickUpStone(){
        myRobot.clawRotation.setPosition(SKYSTONEConstants.right90);
        myRobot.clawServo.setPosition(SKYSTONEConstants.loosen);
        sleep(800);
        myRobot.elevator.setPower(-0.3);
        sleep(700);
        myRobot.elevator.setPower(0);
        myRobot.clawServo.setPosition(SKYSTONEConstants.tighten);
        //sleep(1000);
        //myRobot.clawRotation.setPosition(SKYSTONEConstants.straight);
    }
    void foundationPlaceRed(SKYSTONEClass myRobot) {
        //Grab the foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        sleep(500);
        //Turn the foundation
        //Robot turns clockwise, therefore negative power
        encoderTurnNoStopLeftOnly(-SKYSTONEAutonomousConstants.cFoundationTurn, 1, 3);
        runMotors(0,0);
        //Drive foundation towards wall
        runMotors(-1, -1);
        sleep(1000);
        runMotors(0, 0);
        //Release foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(500);

    }
    void foundationPlaceBlue(SKYSTONEClass myRobot) {
        //Grab the foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        sleep(500);
        //Turn the foundation
        //Robot turns clockwise, therefore negative power
        encoderTurnNoStopRightOnly(SKYSTONEAutonomousConstants.cFoundationTurn, 1, 3);
        runMotors(0,0);
        //Drive foundation towards wall
        runMotors(-1, -1);
        sleep(1000);
        runMotors(0, 0);
        //Release foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(500);

    }
    void getAngleWaitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addData("Angle: ", myRobot.getBackDistance() + "");
                    telemetry.addData("Left Hue: ", getHue(myRobot.frontColor) + "");
                    telemetry.addData("Right Hue: ", getHue(myRobot.backColor) + "");

                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
    String detectFirstStone(boolean isRedSide) {
        String skyStonePosition;//Drive the distance
        double distance;
        int scale;
        if(isRedSide){
            scale = 1;
        }
        else{
            scale = -1;
        }
        distanceEncoderDrive(2,0.25,1,0, myRobot.frontDistance);
        //Detect where the SkyStone is
        float leftHue = getHue(myRobot.frontColor);
        float rightHue = getHue(myRobot.backColor);
        float depotSideHue = isRedSide?leftHue:rightHue;
        float bridgeSideHue = isRedSide?rightHue:leftHue;
        Log.d("Skystone:", "Bridge Side Hue: " + bridgeSideHue + ". Depot Side Hue: " + depotSideHue);
        if(depotSideHue >= 70) {
            skyStonePosition = "Left";
            //First Strafe
        }
        else if(bridgeSideHue >= 70) {
            //First Strafe
            if(isRedSide) {
                encoderStraightDrive(-3, 1);
            }
            encoderStrafeDriveInchesRight(scale*(SKYSTONEAutonomousConstants.doubleAdjustDistance+SKYSTONEAutonomousConstants.doubleCenterDistance), 1);
            if(isRedSide){
                encoderStraightDrive(1, 1);
            }
            else{
                encoderStraightDrive(2, 1);
            }

            skyStonePosition  = "Right";

        }
        else {
            //First strafe
            encoderStrafeDriveInchesRight(scale*SKYSTONEAutonomousConstants.doubleCenterDistance, 1);
            skyStonePosition = "Center";
        }
        Log.d("SkyStone:", "SkyStone Position: " + skyStonePosition);


        return skyStonePosition;
    }
    void grabFoundation(double speed, boolean blue) {
        //Raise up foundation servos
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        if (blue){
            //Turns to match Foundation
            encoderTurn(-173, 1, 2);
            //Let go of stone
            myRobot.frontGrabber.setPosition(SKYSTONEConstants.frUp);
        } else {
            //Turns to match Foundation
            encoderTurn(-178, 1, 2);
            //Let go of stone
            myRobot.frontBase.setPosition(SKYSTONEConstants.flUp);
        }

        //Drive to foundation
        encoderStraightDrive(SKYSTONEAutonomousConstants.foundationDistance, speed);
        runMotors(-0.35, -0.35);
        sleep(200);
        runMotors(0,0);
        //Grab the foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        //encoderStrafeDriveInchesRight(-3, 1);
        sleep(200);
    }
    void fastGrabFoundation(double speed, boolean blue) {
        //Raise up foundation servos
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        if (blue){
            //Turns to match Foundation
            encoderTurn(-173, 1, 2);
            //Let go of stone
            myRobot.frontGrabber.setPosition(SKYSTONEConstants.frUp);
        } else {
            //Turns to match Foundation
            encoderTurn(-178, 1, 2);
            //Let go of stone
            myRobot.frontBase.setPosition(SKYSTONEConstants.flUp);
        }

        //Drive to foundation
        encoderStraightDrive(SKYSTONEAutonomousConstants.foundationDistance, speed);
        runMotors(-0.35, -0.35);
        sleep(200);
        runMotors(0,0);
        //Grab the foundation
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        encoderStrafeDriveInchesRight(-3, 1);
        sleep(200);
    }
    void coordinateDrive(Localizer.Corner corner, double targetX, double targetY, boolean PID, double tolerance){

        switch (corner){
            case LEFT_UP:
        }

    }
    void directionalDrive(double targetX, double targetY, boolean PID, double tolerance, double targetAngle){
        double maxVelocity = 1;
        double velocity = maxVelocity;
        double rotationVelocity = 0;

        double kR = SKYSTONEAutonomousConstants.ddkR;
        double kP = SKYSTONEAutonomousConstants.ddkP;
        double kD = SKYSTONEAutonomousConstants.ddkD;
        double dDistance;
        double dt;
        ElapsedTime clock = new ElapsedTime();
        double t1 = clock.nanoseconds();

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
        xRaw = getXRaw(corner);
        yRaw = getYRaw(corner);
        double previousX = xRaw;
        double previousY = yRaw;
        double xDis = targetX - xRaw;
        double yDis = targetY - yRaw;
        double totalDistance=Math.sqrt(xDis*xDis+yDis*yDis);;
        while((Math.abs(yDis)>tolerance || Math.abs(xDis)>tolerance) && opModeIsActive()){
            currentAngle = loopAround(getHorizontalAngle());
            currentError = targetAngle - currentAngle;
            rotationVelocity = currentError * kR;
            double t2 = clock.nanoseconds();
            dt = t2-t1;

            xDis = targetX - xRaw;
            yDis = targetY - yRaw;
            if(xRaw>250 || yRaw>250){
                Log.d("Skystone: ", "FinalX: "  + xRaw + " FinalY: " + yRaw);
                break;
            }
            telemetry.addData("Skystone: xDis = " + xDis + " yDis = " + yDis,"");
            Log.d("Skystone:", "GyroError: " + currentError + " Rotation Velocity: " + rotationVelocity);
            Log.d("Skystone:"," Targets: targetX = " + targetX + " targetY = " + targetY);
            Log.d("Skystone: ", "xRaw: " + xRaw + " yRaw: " + yRaw);

            if(PID) {
                totalDistance = Math.sqrt(xDis*xDis+yDis*yDis);
                double dYDis = (xRaw-previousX)/dt*Math.pow(10,9);
                double dXDis = (yRaw-previousY)/dt*Math.pow(10,9);

                double xVelocity = Math.min(1, SKYSTONEAutonomousConstants.minimumPower+Math.abs(maxVelocity * getPD(xDis, dXDis, kP, kD)));
                double yVelocity = Math.min(1, SKYSTONEAutonomousConstants.minimumPower+Math.abs(maxVelocity * getPD(yDis, dYDis, kP, kD)));
                velocity = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);

                Log.d("Skystone", "Distance error = " + totalDistance + "Change in xDistance error = " + dXDis + "Change in yDistance error = " + dYDis + "dt = " + dt);
                Log.d("Skystone:", "Proportional Action = " + totalDistance*kP + "DifferentialXAction = " + dXDis*kD + "DifferentialYAction = " + dYDis*kD);
            }
            //store old values
            previousX = xRaw;
            previousY = yRaw;
            t1=t2;

            double angle = Math.atan2(xDis,yDis);
            if(corner == Localizer.Corner.RIGHT_UP || corner == Localizer.Corner.RIGHT_DOWN){
                angle = Math.PI+angle;
            }
            freeDrive(angle, velocity, rotationVelocity);
            Log.d("Skystone: ", "Skystone Angle: "+ (angle*180/Math.PI) + "Velocity: " + velocity+ " RotationVelocity" + rotationVelocity);

        }
    }

    private double getXRaw(Localizer.Corner corner) {
        double xRaw = 0;
        switch (corner){
            case LEFT_DOWN:
                xRaw = myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                break;
            case LEFT_UP:
                xRaw = myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_DOWN:
                xRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_UP:
                xRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                break;
        }
        return xRaw;
    }

    private double getYRaw(Localizer.Corner corner) {
        double yRaw = 0;
        switch (corner){
            case LEFT_DOWN:
                yRaw = myRobot.backDistance.getDistance(DistanceUnit.INCH);
                break;
            case LEFT_UP:
                yRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_DOWN:
                yRaw = myRobot.frontDistance.getDistance(DistanceUnit.INCH);
                break;
            case RIGHT_UP:
                yRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.backDistance.getDistance(DistanceUnit.INCH);
                break;
        }
        return yRaw;
    }

    double getP(double error, double kP){
        return Math.min(1,Math.abs(error)*kP);
    }

    double getPD(double error, double dError, double kP, double kD){
        return Math.min(1, Math.abs(error*kP + dError*kD));
    }

    void freeDrive(double direction, double velocity, double rotationVelocity){
        double s = Math.sin(direction + Math.PI / 4.0);
        double c = Math.cos(direction + Math.PI / 4.0);
        double a = Math.max(Math.abs(s), Math.abs(c));
        s /= a;
        c /= a;

        final double v1 = velocity * s + rotationVelocity;
        final double v2 = velocity * c - rotationVelocity;
        final double v3 = velocity * c + rotationVelocity;
        final double v4 = velocity * s - rotationVelocity;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);
        myRobot.lf.setPower(v1/scale);
        myRobot.rf.setPower(v2/scale);
        myRobot.lb.setPower(v3/scale);
        myRobot.rb.setPower(v4/scale);
        Log.d("Skystone: ", " lfPower: " + v1/scale + " rfPower: " + v2/scale + "lbPower: " + v3/scale + " rbPower: " + v4/scale);
    }

    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }

    //(x1, y1): Coordinates of where you place the stone
    //(x2, y2): Coordinates of where you grab the next stone for depot case
    void placeAndReturn(double x1, double y1, double x2, double y2, String skyStonePosition, boolean isRedSide) {
        double adjustment = 0;
        int multiplier = 1;
        if(!isRedSide){
            multiplier = -1;
        }
        if(skyStonePosition.equals("Center")){
            adjustment = 8;
        }
        else if(skyStonePosition.equals("Bridge")){
            adjustment = 16;
        }
        Log.d("Skystone: ", "Skystoneposition " + skyStonePosition);
        directionalDrive(x1, y1, true, 1,0);
        frontReleaseStone();
        sleep(500);
        frontCarryStone();
        backCarryStone();
        directionalDrive(x1+7, y1-5, true, 1,0);
        int returnDistance = 60;
        straighteningEncoderDrive(returnDistance*multiplier, 0, 50, 1);
        frontReleaseStone();
        directionalDrive(x2, y2 + adjustment, true, 1,0);
        frontGrabStone();
        sleep(250);
        frontCarryStone();
        directionalDrive(x2+7, y2+5 + adjustment, true, 1,0);
        returnDistance = -65;
        straighteningEncoderDrive(returnDistance*multiplier, 0, 50, 1);
    }
    void grabFoundation(boolean isRedSide) {
        int multiplier = 1;
        if(!isRedSide){
            multiplier = -1;
        }
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        frontCarryStone();
        backCarryStone();
        encoderStrafeDriveInchesRight(multiplier*-5,1);
        encoderTurn(multiplier*90, 1, 3);
        distanceEncoderDrive(38,1,1, 90, myRobot.frontDistance);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        sleep(250);
        encoderTurnNoStopPowers(70, -1,-0.5,3, false);
        encoderTurnNoStopLeftOnly(0,1,3);
        encoderStraightDrive(-12,1);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(250);
        encoderStraightDrive(5,1);
    }

    String detectSkyStonePosition(boolean isRedSide) {
        double frontHue = getHue(myRobot.frontColor);
        double backHue = getHue(myRobot.backColor);
        String skyStonePosition = "Center";
        if(frontHue>70){
            skyStonePosition = "Depot";
        }
        else if(backHue>70){
            skyStonePosition = "Bridge";
        }
        Log.d("Skystone: ", "FrontHue: " + frontHue + " BackHue: " + backHue);
        if(isRedSide){
            myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsOpen);
            myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        }
        if(skyStonePosition.equals("Center")){
            encoderStraightDrive(-8, 1);
            //distanceEncoderDrive(baseDistance+8,0.5,1,0, myRobot.frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        if(skyStonePosition.equals("Bridge")){
            encoderStraightDrive(-16,1);
            //distanceEncoderDrive(baseDistance+16,0.5,1,0, myRobot.frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        Log.d("Skystone: ", " skyStonePosition: " + skyStonePosition);
        telemetry.addData("SkystonePosition", skyStonePosition);
        telemetry.update();
        return skyStonePosition;
    }

    void park(boolean isRedSide){
        if (isRedSide){
            encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.foundationClear, 1);
        } else{
            encoderStrafeDriveInchesRight(-SKYSTONEAutonomousConstants.foundationClear, 1);
        }
        encoderStraightDrive(SKYSTONEAutonomousConstants.skyBridgeDrive, 1);
    }
}