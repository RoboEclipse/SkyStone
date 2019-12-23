package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
    BNO055IMU imu;
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
        encoderTurnNoStopPowers(targetAngle, power, power, tolerance);
    }
    void encoderTurnNoStopPowers(double targetAngle, double leftPower, double rightPower, double tolerance) {
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double leftDrivePower = leftPower;
        double rightDrivePower = rightPower;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(-leftDrivePower, rightDrivePower);
        while(Math.abs(error)>tolerance && opModeIsActive()){
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle-currentAngle);
            leftDrivePower = Math.max(Math.min(error/60, 1),-1)*Math.abs(leftPower);
            rightDrivePower = Math.max(Math.min(error/60, 1),-1)*Math.abs(rightPower);
            runMotors(-leftDrivePower, rightDrivePower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + leftDrivePower + "rightPower: " + rightDrivePower + "CurrentAngle: " + currentAngle);
        }
    }
    void turn(double targetAngle, double leftPower, double rightPower, double tolerance){
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double leftDrivePower;
        double rightDrivePower;
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive()){
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle-currentAngle);
            leftDrivePower = Math.max(Math.min(error/60, 1),-1)*Math.abs(leftPower);
            rightDrivePower = Math.max(Math.min(error/60, 1),-1)*Math.abs(rightPower);
            leftDrivePower = floorPower(leftDrivePower, 0.45);
            rightDrivePower = floorPower(rightDrivePower, 0.45);
            runMotors(-leftDrivePower, rightDrivePower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + leftDrivePower + "rightPower: " + rightDrivePower + "CurrentAngle: " + currentAngle);
        }
    }

    void encoderTurnNoStopLeftOnly(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, power, 0, tolerance);
    }

    void encoderTurnNoStopRightOnly(double targetAngle, double power, double tolerance) {
        encoderTurnNoStopPowers(targetAngle, 0, power, tolerance);
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

    double getRoll(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        output = loopAround(output);
        return output;
    }
    double getVerticalAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
                sensor.blue() * scale,
                 values
        );
        return values[0];
    }

    void frontGrabStone (){
        myRobot.frontLower.setPosition(SKYSTONEConstants.frontLow);
        sleep(200);
        myRobot.frontGrabber.setPosition(SKYSTONEConstants.frontGrab);
        sleep(800);
    }
    void frontReleaseStone(){
        myRobot.frontLower.setPosition(SKYSTONEConstants.frontPlace);
        sleep(600);
        myRobot.frontGrabber.setPosition(SKYSTONEConstants.frontLoosen);
        sleep(200);
    }

    //VuforiaDetectionStuff

    //Vuforia Stuff
    /*
     * Initialize the Vuforia localization engine.
     */
    /*
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    */
    // Load the data sets for the trackable objects. These particular data
    /*
     * Initialize the TensorFlow Object Detection engine.

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(/*tfodMonitorViewId*
/*
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
        List<Recognition> runTensorFlow(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                Log.d("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    Log.d(String.format("label (%d)", i), recognition.getLabel());
                    Log.d(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    Log.d(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }
            return updatedRecognitions;
        }
        return null;
    }
    */

    //Complex Methods
    void pickUpStone(){
        myRobot.clawRotation.setPosition(SKYSTONEConstants.right90);
        myRobot.clawServo.setPosition(SKYSTONEConstants.loosen);
        sleep(800);
        myRobot.rightElevator.setPower(-0.3);
        myRobot.leftElevator.setPower(-0.3);
        sleep(700);
        myRobot.rightElevator.setPower(0);
        myRobot.leftElevator.setPower(0);
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
                    telemetry.addData("Left Hue: ", getHue(myRobot.leftColor) + "");
                    telemetry.addData("Right Hue: ", getHue(myRobot.rightColor) + "");

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
        float leftHue = getHue(myRobot.leftColor);
        float rightHue = getHue(myRobot.rightColor);
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
            myRobot.frontLower.setPosition(SKYSTONEConstants.flUp);
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
            myRobot.frontLower.setPosition(SKYSTONEConstants.flUp);
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
    void directionalDrive(double targetX, double targetY, boolean PID, boolean rotation, double targetAngle, double tolerance){
        double xDis = targetX - myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
        double yDis = targetY - myRobot.frontDistance.getDistance(DistanceUnit.INCH);
        double totalDistance;
        double kR = 0.01;
        double velocity = 1;
        double rotationVelocity = 0;
        double kP = 1/20;
        Log.d("Skystone: ", "kP " + kP + " kR " + kR + " tolerance" + tolerance);
        while(yDis>tolerance || xDis>tolerance){
            xDis = targetX - myRobot.elevatorDistance.getDistance(DistanceUnit.INCH);
            yDis = targetY - myRobot.frontDistance.getDistance(DistanceUnit.INCH);
            telemetry.addData("Skystone: xDis = " + xDis + " yDis = " + yDis,7);
            Log.d("Skystone:"," Distance to Wall: xDis = " + xDis + " yDis = " + yDis);
            Log.d("Skystone:"," LeftWallDistance: targetX = " + targetX + " targetY = " + targetY);
            totalDistance = Math.sqrt(xDis*xDis+yDis*yDis);
            if(PID) {
                velocity *= getP(totalDistance, kP);
            }
            if(rotation){
                rotationVelocity = (targetAngle-getHorizontalAngle()) * kR;
            }
            double angle = Math.atan2(xDis,yDis);
            freeDrive(angle, velocity, rotationVelocity);
            Log.d("Skystone: ", "Skystone Angle: "+ angle + "Velocity: " + velocity+ " RotationVelocity" + rotationVelocity);

        }
    }

    double getP(double error, double kP){
        return Math.min(1,Math.abs(error)*kP);
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
    }

    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }
}
