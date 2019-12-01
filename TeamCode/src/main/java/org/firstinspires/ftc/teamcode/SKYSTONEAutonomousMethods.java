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
    void encoderStraightDriveInches(double inches, double power){
        encoderStraightDriveNoStop(inches, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void encoderStraightDriveNoStop(double inches, double power) {
        ElapsedTime time = new ElapsedTime();
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                myRobot.lf.setPower(Math.max(Math.abs(lfError/500), 0.1)*lfError/Math.abs(lfError));
            }
            else{
                myRobot.lf.setPower(0);
                reached = false;
            }
            if(Math.abs(lbError)>20){
                myRobot.lb.setPower(Math.max(Math.abs(lbError/500), 0.1)*lbError/Math.abs(lbError));
            }
            else{
                myRobot.lb.setPower(0);
                reached = false;
            }
            if(Math.abs(rfError)>20){
                myRobot.rf.setPower(Math.max(Math.abs(rfError/500), 0.1)*rfError/Math.abs(rfError));
            }
            else{
                myRobot.rf.setPower(0);
                reached = false;
            }
            if(Math.abs(rbError)>20){
                myRobot.rb.setPower(Math.max(Math.abs(rbError/500), 0.1)*rbError/Math.abs(rbError));
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
    void distanceStraightDriveNoStop(double inches, double power) {
        ElapsedTime time = new ElapsedTime();
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);

        while (notCloseEnough(20, myRobot.lf, myRobot.rf, myRobot.lb, myRobot.rb) && time.milliseconds()<4000 && opModeIsActive()){
            myRobot.lf.setPower(power);
            myRobot.lb.setPower(-power);
            myRobot.rf.setPower(-power);
            myRobot.rb.setPower(power);
            Log.d("Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", myRobot.rb.getCurrentPosition()+"");
        }
    }

    //Negative = Left, Positive = Right
    void distanceStrafeDriveInchesRight(double inches, double power){
        distanceStraightDriveNoStop(inches, power);
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
        double currentAngle = getHorizontalAngle();
        //double startDifference = currentAngle-targetAngle;
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(-drivePower, drivePower);
        while(Math.abs(error)>tolerance && opModeIsActive()){

            currentAngle = getHorizontalAngle();
            /*
            if(power>0){
                drivePower = 0.1 + error/startDifference*power*0.9;
            }
            else {
                drivePower = -0.1 + error/startDifference*power*0.9;
            }*
             */
            error = loopAround(targetAngle-currentAngle);
            drivePower = Math.max(Math.min(error/60, 1),-1)*Math.abs(power);
            runMotors(-drivePower, drivePower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " Adjust: " + drivePower + "CurrentAngle: " + currentAngle);
        }
    }

    void encoderTurnNoStopLeftOnly(double targetAngle, double power, double tolerance) {
        double currentAngle = getHorizontalAngle();
        //double startDifference = currentAngle-targetAngle;
        double error = targetAngle - currentAngle;
        error = loopAround(error);
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(-drivePower, 0);
        while (Math.abs(error) > tolerance && opModeIsActive()) {
            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle - currentAngle);
            drivePower = Math.max(Math.min(error / 60, 1), -1) * Math.abs(power);
            runMotors(-drivePower, 0);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " Adjust: " + drivePower + "CurrentAngle: " + currentAngle);
        }
    }

    void encoderTurnNoStopRightOnly(double targetAngle, double power, double tolerance) {
        double currentAngle = getHorizontalAngle();
        //double startDifference = currentAngle-targetAngle;
        double error = targetAngle - currentAngle;
        error = loopAround(error);
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(0, drivePower);
        while (Math.abs(error) > tolerance && opModeIsActive()) {

            currentAngle = getHorizontalAngle();
            error = loopAround(targetAngle - currentAngle);
            drivePower = Math.max(Math.min(error / 60, 1), -1) * Math.abs(power);
            runMotors(0, drivePower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " Adjust: " + drivePower + "CurrentAngle: " + currentAngle);
        }
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

    void backDistanceEncoderDrive(double distance, double tolerance, double power){
        backDistanceEncoderDriveNoStop(distance, tolerance, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //TODO: Investigate how the changes effect these
    void backDistanceEncoderDriveNoStop(double distance, double tolerance, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(power, power);
        double curDistance = myRobot.getBackDistance();
        //double startDistance = curDistance;
        double error = curDistance-distance;
        double adjust;
        while (Math.abs(error)>tolerance){
            /*
            adjust = 0.1*error/Math.abs(error) + 0.9*(Math.min(Math.abs(error),10))/10*power;
            if(error*power<0){
                adjust*=-1;
            }
             */
            error = curDistance-distance;
            adjust = Math.max(Math.min(error, 20),-20)/20*power;
            runMotors(adjust, adjust);
            curDistance = myRobot.getBackDistance();
            Log.d("Skystone: ", "DistanceDrive Error: " + error + " Adjust: " + adjust + "CurrentDistance: " + curDistance);
        }
    }
    void straighteningEncoderDriveInches(double inches, double targetAngle, double tolerance, double power){
        straighteningEncoderDriveInchesNoStop(inches, targetAngle, tolerance, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void straighteningEncoderDriveInchesNoStop(double inches, double targetAngle, double tolerance, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private double getCorrection(double currentAngle, double targetAngle){
        double errorAngle = loopAround(targetAngle-currentAngle);
        double PCoefficient = 1.0/10;
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
            adjust = Math.max(Math.min(error, 20),-20)/20*power;
            //Ensure power is above 0.2
            if(adjust > 0){
                adjust = Math.max(adjust, 0.2);
            }
            else{
                adjust = Math.min(adjust, -0.2);
            }
            runMotors(adjust - steer, adjust + steer);
            curDistance = sensor.getDistance(DistanceUnit.INCH);
            Log.d("Skystone: ", "FrontDistanceDrive Error: " + error + " Adjust: " + adjust + "CurrentDistance: " + curDistance + "Steer: " + steer);
        }
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
    float hsv(ColorSensor sensor){
        float[] values = new float[3];
        int scale = 255;
        Color.RGBToHSV(sensor.red()*scale,
                sensor.green() * scale,
                sensor.blue() * scale,
                 values
        );
        return values[0];
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
        myRobot.clawServo.setPosition(SKYSTONEConstants.autoLoosen);
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
        runMotors(-0.6, -0.6);
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
        encoderTurnNoStopRightOnly(-SKYSTONEAutonomousConstants.cFoundationTurn, 1, 3);
        runMotors(0,0);
        //Drive foundation towards wall
        runMotors(-0.6, -0.6);
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
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
    String pickUpFirstStoneAndTurn() {
        String skyStonePosition;//Drive the distance
        distanceEncoderDrive(1.9,0.3,1,0, myRobot.frontDistance);
        //Detect where the SkyStone is
        float leftHue = hsv(myRobot.leftColor);
        float rightHue = hsv(myRobot.rightColor);
        if(leftHue >= 100) {
            skyStonePosition = "Left";
            //First Strafe
        }
        else if(rightHue >= 100) {
            //First Strafe
            encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.doubleAdjustDistance+SKYSTONEAutonomousConstants.doubleCenterDistance+2, 1);
            skyStonePosition  = "Right";

        }
        else {
            //First strafe
            encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.doubleCenterDistance, 1);
            skyStonePosition = "Center";
        }
        Log.d("SkyStone Position: ", skyStonePosition);
        //Grab the stone
        myRobot.leftClaw.setPosition(SKYSTONEConstants.flDown);
        sleep(800);
        //Drive backwards
        encoderStraightDriveInches(-4, 1);
        //Turn
        encoderTurn(-88, 1.0, 1);
        return skyStonePosition;
    }
}
