package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;
import java.util.Locale;

import static java.lang.Thread.sleep;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubMotor;




public class SKYSTONEClass extends SKYSTONEDrivetrainClass{
    //Hardware
    DcMotor clawSlide, leftElevator, rightElevator;
    Servo clawRotation, leftFoundationServo, rightFoundationServo, clawServo, leftClaw, rightClaw, capServo;
    CRServo collectionRotationServo;
    DistanceSensor frontDistance, rightDistance, backLeftDistance, backRightDistance, backDistance, elevatorDistance;
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    ExpansionHubMotor lbBR, lfBR, rbBR, rfBR;

    MotionProfile1D mp;
    //Software
    private Telemetry telemetry;

    //Classes
    private SKYSTONEConfiguration skystoneNames = new SKYSTONEConfiguration();
    //SKYSTONEDrivetrainClass drivetrain = new SKYSTONEDrivetrainClass();
    //private SKYSTONEConstants skystoneConstants = new SKYSTONEConstants();

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //HardwareMaps
        initializeDriveTrain(hardwareMap, telemetry_);
        /*
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        */
        clawRotation = hardwareMap.servo.get(skystoneNames.rotationServo);
        clawSlide = hardwareMap.dcMotor.get(skystoneNames.slidingMotor);
        leftElevator = hardwareMap.dcMotor.get(skystoneNames.leftElevatorMotor);
        rightElevator = hardwareMap.dcMotor.get(skystoneNames.rightElevatorMotor);
        leftFoundationServo = hardwareMap.servo.get(skystoneNames.leftFoundationServo);
        rightFoundationServo = hardwareMap.servo.get(skystoneNames.rightFoundationServo);
        capServo = hardwareMap.servo.get(skystoneNames.cappingServo);
        clawServo = hardwareMap.servo.get(skystoneNames.collectorServo);
        leftClaw = hardwareMap.servo.get(skystoneNames.leftClaw);
        rightClaw = hardwareMap.servo.get(skystoneNames.rightClaw);
        collectionRotationServo = hardwareMap.crservo.get(skystoneNames.collectorRotationServo);
        backDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backDistance);
        elevatorDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.elevatorHeight);
        frontDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.leftDistance);
        //rightDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.rightDistance);
        //backLeftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backLeftDistance);
        //backRightDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backRightDistance);
        //Motor Settings
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    //Methods
    //Shortcuts
    private void setModeAllDrive(DcMotor.RunMode mode){
        lb.setMode(mode);
        lf.setMode(mode);
        rb.setMode(mode);
        rf.setMode(mode);
    }
    private void multiSetTargetPosition(double ticks, DcMotor...motors){
        for(DcMotor motor:motors){
            motor.setTargetPosition((int) Math.round(ticks));
        }
    }
    private boolean anyBusy(){
        return lb.isBusy() || lf.isBusy() || rb.isBusy() || rf.isBusy();
    }
    void runMotors (double leftPower, double rightPower){
        lb.setPower(leftPower);
        lf.setPower(leftPower);
        rb.setPower(rightPower);
        rf.setPower(rightPower);
    }

    void readEncodersBR(){
        bulkData = expansionHub.getBulkInputData();
        telemetry.addData(
                "Encoders from Bulk Read", "lf: " + bulkData.getMotorCurrentPosition(lfBR)
                        + " lb: " + bulkData.getMotorCurrentPosition(lbBR)
                        + " rf: " + bulkData.getMotorCurrentPosition(rfBR)
                        + " rb: "+ bulkData.getMotorCurrentPosition(rbBR)
        );

    }

    //Drive Stuff
    //Preferably Do Not Touch
    void drive(double direction, double velocity, double rotationVelocity) {
        SKYSTONEClass.Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        rf.setPower(w.rf);
        lb.setPower(w.lr);
        rb.setPower(w.rr);
        telemetry.addData("Powers", String.format(Locale.US, "lf %.2f lr %.2f rf %.2f rr %.2f", w.lf, w.lr, w.rf, w.rr));
    }
    private static class Wheels {
        double lf, lr, rf, rr;

        Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }
    private SKYSTONEClass.Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        //final double vd = velocity;
        //final double td = direction;
        //final double vt = rotationVelocity;

        double s = Math.sin(direction + Math.PI / 4.0);
        double c = Math.cos(direction + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = velocity * s + rotationVelocity;
        final double v2 = velocity * c - rotationVelocity;
        final double v3 = velocity * c + rotationVelocity;
        final double v4 = velocity * s - rotationVelocity;

        // Ensure that none of the values go over 1.0. If none of the provided values are
        // over 1.0, just scale by 1.0 and keep all values.
        double scale = ma(1.0, v1, v2, v3, v4);

        return new SKYSTONEClass.Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }
    private static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) {
            ret = Math.max(ret, Math.abs(x));
        }
        return ret;
    }



    //Drivetrain
    void readEncoders(){
        telemetry.addData(
                "Encoders", "lf: " + lf.getCurrentPosition()
                        + " lb: " + lb.getCurrentPosition()
                        + " rf: " + rf.getCurrentPosition()
                        + " rb: "+ rb.getCurrentPosition()
                        + " left elevator: " + leftElevator.getCurrentPosition()
                        + " right elevator: " + rightElevator.getCurrentPosition()
                        + " slide motor: " + clawSlide.getCurrentPosition()
        );
    }


    //Servo Movement
    void grabStones (double closingDegrees) { clawServo.setPosition(closingDegrees); }
    void rotateStackingClaw(double turningDegrees) { clawRotation.setPosition(turningDegrees); }
    void runCollectorServos(double collectorPower){
        collectionRotationServo.setPower(collectorPower);
    }
    void setCapServo (double turningDegrees) { capServo.setPosition(turningDegrees); }
    void moveFrontClaw (double position){
        leftClaw.setPosition(position);
    }
    //Motor Movement
    void runElevatorMotors(double power){
        leftElevator.setPower(power);
        rightElevator.setPower(power);
    }
    void runWithEncoder(double power, int ticks, DcMotor...motors){
        ElapsedTime time = new ElapsedTime();
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while(anyBusy(5, ticks, motors) && time.milliseconds()<2500){
            for(DcMotor motor : motors){
                Log.d("Motor " + motor.getPortNumber(), motor.getCurrentPosition() + "");
            }
        }
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    void runWithEncoderBegin(double power, int ticks, DcMotor...motors){
        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
    }

    void runWithEncoderEnd(int ticks, DcMotor...motors){
        ElapsedTime time = new ElapsedTime();
        while(anyBusy(5, ticks, motors) && time.milliseconds()<2500){
            for(DcMotor motor : motors){
                Log.d("Skystone Motor " + motor.getPortNumber(), motor.getCurrentPosition() + "");
            }
        }
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }



    boolean anyBusy(int tolerance, int targetPosition, DcMotor...motors){
        for(DcMotor motor : motors){
            if(Math.abs(targetPosition-motor.getCurrentPosition())>tolerance){
                return true;
            }
        }
        return false;
    }

    double getBackDistance(){
        return backDistance.getDistance(DistanceUnit.INCH);
    }
    double getFrontDistance() {
        return frontDistance.getDistance(DistanceUnit.INCH);
    }
    double getElevatorDistance(){
        return elevatorDistance.getDistance(DistanceUnit.CM);
    }
    void elevatorDistanceDrive(double power, int ticks, double distance, double tolerance){
        /*
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        multiSetTargetPosition(ticks, leftElevator, rightElevator);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        leftElevator.setPower(power);
        rightElevator.setPower(power);
        double error = Math.abs(getElevatorDistance()-distance);
        while (error>tolerance){
            error = Math.abs(getElevatorDistance()-distance);
            Log.d("ElevatorError: ", error + "");
        }
        leftElevator.setPower(0);
        rightElevator.setPower(0);
        leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void resetAutonomous() {
        clawRotation.setPosition(SKYSTONEConstants.straight);
        clawServo.setPosition(SKYSTONEConstants.tighten);
        clawSlide.setPower(0.3);
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        clawSlide.setPower(0);
        //TODO: Replace with driveWithEncoder
        //leftElevator.setTargetPosition(SKYSTONEConstants.startingElevatorHeight);
        //rightElevator.setTargetPosition(SKYSTONEConstants.startingElevatorHeight);
    }

    String getSkystonePosition(SKYSTONEVuforiaDetection vuforiaMethods, List<VuforiaTrackable> detections) {
        String skyStonePosition = "Left";
        double y = vuforiaMethods.loopDetection(telemetry, detections);
        telemetry.addData("Offset: ", y);
        if(y > SKYSTONEConstants.stoneDiff){
            skyStonePosition = "Right";
            Log.d("SkystonePosition", "Right: " + y);
            telemetry.addData("SkystonePosition", "Right");
        }
        else if(Math.abs(y)< SKYSTONEConstants.stoneDiff){
            skyStonePosition = "Center";
            Log.d("SkystonePosition", "Center: " + y);
        }
        else{
            Log.d("SkyStonePosition", "Left: " + y);
        }
        return skyStonePosition;
    }

    //move foundation for endgame
    void motionProfileFoundationMove(DcMotor...motors){
        //constants
        double distance = 30;
        int ticks = (int) (distance * SKYSTONEConstants.TICKS_PER_INCH);

        //create motion profile (units are inches or inches/second)
        mp = new MotionProfile1D(distance, 10, 1, 0, 0);

        //set motors to encoder mode
        ElapsedTime time = new ElapsedTime();
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //set power of motors based on motion profile
        while(anyBusy(5, ticks, motors) && time.milliseconds()<2500){
            double averageDistance = 0;
            for (DcMotor motor : motors){
                averageDistance += motor.getCurrentPosition() / SKYSTONEConstants.TICKS_PER_INCH;
                motor.setPower(mp.currentVelocity / SKYSTONEConstants.maxSpeed);
            }
            averageDistance/=4;
            mp.loop(time.seconds(), averageDistance);
        }

        //stop using encoders
        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


}
