package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    private SKYSTONEDrivetrainClass drivetrain = myRobot;
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
        multiSetTargetPosition(inches* SKYSTONEConstants.TICKS_PER_INCH, drivetrain.lb, drivetrain.lf, drivetrain.rb, drivetrain.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(20, drivetrain.lf, drivetrain.rf, drivetrain.lb, drivetrain.rb) && time.milliseconds()<4000 && opModeIsActive()){
            Log.d("Left Front: ", drivetrain.lf.getCurrentPosition()+"");
            Log.d("Left Back: ", drivetrain.lb.getCurrentPosition()+"");
            Log.d("Right Front: ", drivetrain.rf.getCurrentPosition()+"");
            Log.d("Right Back: ", drivetrain.rb.getCurrentPosition()+"");
        }
    }

    //Negative = Left, Positive = Right
    void encoderStrafeDriveInchesRight(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.lf.setTargetPosition((int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        drivetrain.lb.setTargetPosition(-(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        drivetrain.rf.setTargetPosition(-(int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        drivetrain.rb.setTargetPosition((int) Math.round(inches*SKYSTONEConstants.TICKS_PER_INCH));
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (notCloseEnough(8, drivetrain.lf, drivetrain.lb, drivetrain.rf, drivetrain.rb) && opModeIsActive()){
            Log.d("SkyStone Left Front: ", drivetrain.lf.getCurrentPosition()+"");
            Log.d("SkyStone Left Back: ", drivetrain.lb.getCurrentPosition()+"");
            Log.d("SkyStone Right Front: ", drivetrain.rf.getCurrentPosition()+"");
            Log.d("SkyStone Right Back: ", drivetrain.rb.getCurrentPosition()+"");
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
        double currentAngle = getHorizontalAngle();
        //double startDifference = currentAngle-targetAngle;
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(drivePower, -drivePower);
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
            drivePower = Math.max(Math.min(error/90, 1),-1)*Math.abs(power);
            runMotors(drivePower, -drivePower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " Adjust: " + drivePower + "CurrentAngle: " + currentAngle);
        }
    }

    int leftFrontEncoder(){
        return drivetrain.lf.getCurrentPosition();
    }
    int leftBackEncoder(){
        return drivetrain.lb.getCurrentPosition();
    }
    int rightFrontEncoder(){
        return drivetrain.rf.getCurrentPosition();
    }
    int rightBackEncoder(){
        return drivetrain.rb.getCurrentPosition();
    }

    void backDistanceEncoderDrive(double distance, double tolerance, double power){
        backDistanceEncoderDriveNoStop(distance, tolerance, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

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
    void straighteningEncoderDriveInches(double distance, double targetAngle, double tolerance, double power){
        straighteningEncoderDriveInchesNoStop(distance, targetAngle, tolerance, power);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void straighteningEncoderDriveInchesNoStop(double distance, double targetAngle, double tolerance, double power) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(power, power);
        double curDistance = leftFrontEncoder();
        double targetDistance = distance* SKYSTONEConstants.TICKS_PER_INCH;
        //double startDistance = curDistance;
        double errorDistance = targetDistance-curDistance;
        double adjust;
        while (Math.abs(errorDistance)>tolerance){
            double currentAngle = getHorizontalAngle();
            double steer = getCorrection(currentAngle, targetAngle);
            adjust = Math.max(Math.min(errorDistance, 20),-20)/20*power;
            double leftSpeed = adjust + steer;
            double rightSpeed = adjust - steer;
            errorDistance = targetDistance-curDistance;
            runMotors(leftSpeed, rightSpeed);
            curDistance = leftFrontEncoder();
            Log.d("Skystone: ", "DistanceDrive Error: " + errorDistance +
                    " Angle Steer: " + steer + "CurrentDistance: " + curDistance + "Power: " + adjust + "Angle" + currentAngle);
        }
    }

    private double getCorrection(double currentAngle, double targetAngle){
        double errorAngle = loopAround(targetAngle-currentAngle);
        double PCoefficient = 1.0/30;
        return errorAngle*PCoefficient;
    }

    void frontDistanceEncoderDrive(double distance, double tolerance, double power, double targetAngle){
        frontDistanceEncoderDriveNoStop(distance, tolerance, power, targetAngle);
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void frontDistanceEncoderDriveNoStop(double distance, double tolerance, double power, double targetAngle) {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(power, power);
        double curDistance = myRobot.getLeftDistance();
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
            double curAngle = getHorizontalAngle();
            //double steer = getCorrection(curAngle, targetAngle);
            double steer = 0;
            error = curDistance-distance;
            adjust = -Math.max(Math.min(error, 20),-20)/20*power;
            runMotors(adjust + steer, adjust - steer);
            curDistance = myRobot.getLeftDistance();
            Log.d("Skystone: ", "FrontDistanceDrive Error: " + error + " Adjust: " + adjust + "CurrentDistance: " + curDistance);
        }
    }
    //Attachments


    //private void
    //Shortcuts
    private void setModeAllDrive(DcMotor.RunMode mode){
        drivetrain.lb.setMode(mode);
        drivetrain.lf.setMode(mode);
        drivetrain.rb.setMode(mode);
        drivetrain.rf.setMode(mode);
    }
    private void multiSetTargetPosition(double ticks, DcMotor...motors){
        for(DcMotor motor:motors){
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }
    private boolean anyBusy(){
        return drivetrain.lb.isBusy() || drivetrain.lf.isBusy() || drivetrain.rb.isBusy() || drivetrain.rf.isBusy();
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
        drivetrain.lb.setPower(leftPower);
        drivetrain.lf.setPower(leftPower);
        drivetrain.rb.setPower(rightPower);
        drivetrain.rf.setPower(rightPower);
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

}
