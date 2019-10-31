package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class SKYSTONEClass {
    //Hardware
    DcMotor lb, lf, rb, rf, clawSlide, leftElevator, rightElevator;
    Servo clawRotation, leftFoundationServo, rightFoundationServo, clawServo;
    CRServo collectionRotationServo;
    DistanceSensor leftDistance, rightDistance, backLeftDistance, backRightDistance, backDistance, elevatorDistance;
    //Software
    private Telemetry telemetry;

    //Classes
    private SKYSTONEConfiguration skystoneNames = new SKYSTONEConfiguration();
    //private SKYSTONEConstants skystoneConstants = new SKYSTONEConstants();

    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry_){
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //HardwareMaps
        lb = hardwareMap.dcMotor.get(skystoneNames.backLeftMotor);
        lf = hardwareMap.dcMotor.get(skystoneNames.frontLeftMotor);
        rb = hardwareMap.dcMotor.get(skystoneNames.backRightMotor);
        rf = hardwareMap.dcMotor.get(skystoneNames.frontRightMotor);
        clawRotation = hardwareMap.servo.get(skystoneNames.rotationServo);
        clawSlide = hardwareMap.dcMotor.get(skystoneNames.slidingMotor);
        leftElevator = hardwareMap.dcMotor.get(skystoneNames.leftElevatorMotor);
        rightElevator = hardwareMap.dcMotor.get(skystoneNames.rightElevatorMotor);
        leftFoundationServo = hardwareMap.servo.get(skystoneNames.leftFoundationServo);
        rightFoundationServo = hardwareMap.servo.get(skystoneNames.rightFoundationServo);
        clawServo = hardwareMap.servo.get(skystoneNames.collectorServo);
        collectionRotationServo = hardwareMap.crservo.get(skystoneNames.collectorRotationServo);
        backDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backDistance);
        elevatorDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.elevatorHeight);
        //leftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.leftDistance);
        //rightDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.rightDistance);
        //backLeftDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backLeftDistance);
        //backRightDistance = hardwareMap.get(DistanceSensor.class, skystoneNames.backRightDistance);
        //Motor Settings
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
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
    private void runMotors (double leftPower, double rightPower){
        lb.setPower(leftPower);
        lf.setPower(leftPower);
        rb.setPower(rightPower);
        rf.setPower(rightPower);
    }
    //Drive Stuff
    //Preferably Do Not Touch
    void drive(double direction, double velocity, double rotationVelocity) {
        SKYSTONEClass.Wheels w = getWheels(direction, velocity, rotationVelocity);
        lf.setPower(w.lf);
        rf.setPower(w.rf);
        lb.setPower(w.lr);
        rb.setPower(w.rr);
        telemetry.addData("Powers", String.format(Locale.US, "%.2f %.2f %.2f %.2f", w.lf, w.rf, w.lr, w.rr));
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
    boolean anyBusy(int tolerance, int targetPosition, DcMotor...motors){
        for(DcMotor motor : motors){
            if(Math.abs(targetPosition-motor.getCurrentPosition())>tolerance){
                return true;
            }
        }
        return false;
    }

    double getBackDistance(){
        return backDistance.getDistance(DistanceUnit.CM);
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
        leftElevator.setTargetPosition(SKYSTONEConstants.startingElevatorHeight);
        rightElevator.setTargetPosition(SKYSTONEConstants.startingElevatorHeight);
    }
}
