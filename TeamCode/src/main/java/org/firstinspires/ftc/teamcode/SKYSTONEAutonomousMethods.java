package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

abstract class SKYSTONEAutonomousMethods extends LinearOpMode {
    //Hardware
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    //Software
    private Telemetry telemetry;

    //Classes
    private SKYSTONEClass myRobot = new SKYSTONEClass();
    private SKYSTONEConfiguration skystoneNames = new SKYSTONEConfiguration();
    private SKYSTONEConstants skystoneConstants = new SKYSTONEConstants();

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
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
*/
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
    }
    void waitForStart2(){
        ElapsedTime time  = new ElapsedTime();
        while (!isStarted()){
            double clock = time.seconds();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Time", clock);
            telemetry.update();
        }
    }

    //Methods
    void encoderStraightDriveInches(double inches, double power){
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        multiSetTargetPosition(inches*SKYSTONEConstants.TICKS_PER_INCH, myRobot.lb, myRobot.lf, myRobot.rb, myRobot.rf);
        setModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        runMotors(power, power);
        while (anyBusy() /*&& opModeisActive()*/){
            telemetry.addData("Left Front: ", myRobot.lf.getCurrentPosition());
            telemetry.addData("Left Back: ", myRobot.lb.getCurrentPosition());
            telemetry.addData("Right Front: ", myRobot.rf.getCurrentPosition());
            telemetry.addData("Right Back: ", myRobot.rb.getCurrentPosition());
            telemetry.update();
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        while (anyBusy() /*&& opModeisActive()*/){
            telemetry.addData("Left Front: ", myRobot.lf.getCurrentPosition());
            telemetry.addData("Left Back: ", myRobot.lb.getCurrentPosition());
            telemetry.addData("Right Front: ", myRobot.rf.getCurrentPosition());
            telemetry.addData("Right Back: ", myRobot.rb.getCurrentPosition());
            telemetry.update();
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Positive = Clockwise, Negative = Counterclockwise
    void encoderTurn(double targetAngle, double power, double tolerance){
        double currentAngle = getHorizontalAngle();
        double startDifference = targetAngle-currentAngle;
        double currentDifference = startDifference;
        double drivePower = power;
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        runMotors(drivePower, -drivePower);
        while(getHorizontalAngle()<targetAngle-tolerance || getHorizontalAngle()>targetAngle+tolerance /*&& opModeisActive()*/){
            currentAngle = getHorizontalAngle();
            currentDifference = targetAngle-currentAngle;
            drivePower = 0.1 + Math.abs(currentDifference/startDifference)*power*0.9;
            runMotors(drivePower, -drivePower);
        }
        runMotors(0,0);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    //Attachments


    //private void
    //Shortcuts
    private void setModeAllDrive(DcMotor.RunMode mode){
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
        if(output>180){
            output-=360;
        }
        if(output<-180){
            output+=360;
        }
        return output;
    }
    double getRoll(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.secondAngle;
        if(output>180){
            output-=360;
        }
        if(output<-180){
            output+=360;
        }
        return output;
    }
    double getVerticalAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double output = angles.thirdAngle;
        if(output>180){
            output-=360;
        }
        if(output<-180){
            output+=360;
        }
        return output;
    }
    boolean opModeStatus(){
        if(opModeIsActive()){
            return true;
        }
        else return false;
    }
    //VuforiaDetectionStuff

    //Vuforia Stuff
    /**
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
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }
            return updatedRecognitions;
        }
        return null;
    }
    */

}
