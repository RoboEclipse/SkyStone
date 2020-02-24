package org.firstinspires.ftc.teamcode;

        import android.graphics.Color;
        import android.util.Log;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
        import org.openftc.revextensions2.RevBulkData;

abstract class SKYSTONEAutonomousMethods extends LinearOpMode {
    //Hardware
    // The IMU sensor object
    //BNO055IMU imu;
    private Orientation angles;
    //Software
    //private Telemetry telemetry;

    private int turnPValue = 15;
    //Classes

    SKYSTONEClass myRobot = new SKYSTONEClass();
    Localizer localizer;
    LocalizerReader localizerReader;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    //Backend
    void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        myRobot.initialize(hardwareMap, telemetry);
        localizer = new Localizer(myRobot);
        localizerReader = LocalizerReader.INSTANCE;
        localizerReader.setLocalizer(localizer);
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
        ElapsedTime killTimer = new ElapsedTime();
        runMotors(power, power);
        while (notCloseEnough(20, myRobot.lf, myRobot.lb, myRobot.rf, myRobot.rb) && opModeIsActive() && killTimer.seconds()<2){
            Log.d("SkyStone Left Front: ", myRobot.lf.getCurrentPosition()+"");
            Log.d("SkyStone Left Back: ", myRobot.lb.getCurrentPosition()+"");
            Log.d("SkyStone Right Front: ", myRobot.rf.getCurrentPosition()+"");
            Log.d("SkyStone Right Back: ", myRobot.rb.getCurrentPosition()+"");
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
        double leftProportionalPower;
        double rightProportionalPower;
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
        double currentAngle = getHorizontalAngle();
        double error = targetAngle-currentAngle;
        error = loopAround(error);
        ElapsedTime killTimer = new ElapsedTime();
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Math.abs(error)>tolerance && opModeIsActive() && killTimer.seconds()<3){
            currentAngle = getHorizontalAngle();
            error = targetAngle-currentAngle;
            error = loopAround(error);
            runMotors(leftPower, rightPower);
            Log.d("Skystone: ", "encoderTurn Error: " + error + " leftPower: " + leftPower + "rightPower: " + rightPower + "CurrentAngle: " + currentAngle);
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
    void frontCarryStone () {
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbUp);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
    }
    void backCarryStone () {
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
    }
    //Basic grab
    void frontGrabStone (){
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbDown);
        sleep(250);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
        sleep(250);
    }
    void backGrabStone (){
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbDown);
        sleep(250);
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
        sleep(250);
    }
    //frontRelease is both the release and the starting position before grab
    void backReadyStone(){
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsReady);
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbReady);
    }
    //frontRelease is both the release and the starting position before grab
    void frontReadyStone(){
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsReady);
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbReady);
    }
    void backReleaseStone(){
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbReady);
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsReady);
        sleep(300);
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbStartPosition);
        sleep(100);

    }
    void frontReleaseStone(){
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbReady);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsReady);
        sleep(300);
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbStartPosition);
        sleep(100);
    }

    //VuforiaDetectionStuff

    //Vuforia Stuff

    //Complex Methods
    void getAngleWaitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addData("Angle: ",  getHorizontalAngle() +  "");
                    telemetry.addData("Left Hue: ", getHue(myRobot.frontColor) + "");
                    telemetry.addData("Right Hue: ", getHue(myRobot.backColor) + "");
                    telemetry.addData("Left Distance: ", myRobot.getLeftDistance() + "");
                    telemetry.addData("Front Distance: ", myRobot.getFrontDistance() + "");
                    telemetry.addData("Back Distance: ", myRobot.getBackDistance()+ "");

                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }
    void directionalDrive(double targetX, double targetY, boolean PID, double tolerance, double targetAngle){

        double maxVelocity = 1;
        double velocity = maxVelocity;
        double rotationVelocity = 0;

        double currentAngle;
        double currentError;

        double dt;

        double t1 = 0;
        double xDis = 0;
        double yDis = 0;
        double totalDistance;
        RevBulkData prevData = myRobot.expansionHub.getBulkInputData();
        ElapsedTime clock = new ElapsedTime();
        if(opModeIsActive()){
            t1 = clock.nanoseconds();
            xDis = targetX - localizer.getX();
            yDis = targetY - localizer.getY();
            Log.d("Skystone: ",
                    "Direct Drive Target:(" + targetX + "," + targetY
                            + " tolerance" + tolerance + " Start At: ("
                            + localizer.getX() + "," + localizer.getY() + ")");

        }
        while((Math.abs(yDis)>tolerance || Math.abs(xDis)>tolerance) && opModeIsActive()){
            double kR;
            double kP;
            double kD;
            if(localizer.getEncoderOnly()){
                kR = SKYSTONEAutonomousConstants.ekR;
                kP = SKYSTONEAutonomousConstants.ekP;
                kD = SKYSTONEAutonomousConstants.ekD;
            }
            else{
                kR = SKYSTONEAutonomousConstants.ddkR;
                kP = SKYSTONEAutonomousConstants.ddkP;
                kD = SKYSTONEAutonomousConstants.ddkD;
            }
            //Start of update
            RevBulkData encoderData = myRobot.expansionHub.getBulkInputData();
            currentAngle = loopAround(getHorizontalAngle());
            currentError = targetAngle - currentAngle;
            rotationVelocity = currentError * kR;
            localizer.update(prevData, encoderData);

            //End of update
            double t2 = clock.nanoseconds();
            dt = t2-t1;
            xDis = targetX - localizer.getX();
            yDis = targetY - localizer.getY();

            Log.d("Skystone:", "GyroError: " + rotationVelocity);
            double angle = 0;
            if(PID) {
                totalDistance = Math.sqrt(xDis*xDis+yDis*yDis);
                //double dXDis = (xRaw-previousX)/dt*Math.pow(10,9);
                //double dYDis = (yRaw-previousY)/dt*Math.pow(10,9);

                double dY = localizer.getdY(encoderData);
                double dX = localizer.getdX(encoderData);

                double xVelocity = maxVelocity * getPD(xDis, dX, kP, kD);
                double yVelocity = maxVelocity * getPD(yDis, dY, kP, kD);
                angle = Math.atan2(xVelocity * SKYSTONEAutonomousConstants.lateralFactor, yVelocity);

                velocity = Math.min (1, SKYSTONEAutonomousConstants.minimumPower + Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity));

                Log.d("Skystone", "Distance error = " + totalDistance + " Change in xDistance error = " + dX + " Change in yDistance error = " + dY + " dt = " + dt);
                Log.d("Skystone:", " Proportional Action = " + totalDistance*kP + " xVelocity = " + xVelocity + " yVelocity = " + yVelocity);
                Log.d("Skystone", " kP: " + kP + "kD: " + kD + " kR: " + kR);
            } else{
                angle = Math.atan2(xDis * SKYSTONEAutonomousConstants.lateralFactor ,yDis);
            }
            t1=t2;

            if(localizer.getCorner() == Localizer.Corner.RIGHT_UP
                    || localizer.getCorner() == Localizer.Corner.RIGHT_DOWN){
                angle = Math.PI+angle;
            }
            freeDrive(angle, velocity, rotationVelocity);
            Log.d("Skystone: ", "Skystone Angle: "+ (angle*180/Math.PI) + "Velocity: " + velocity+ " RotationVelocity" + rotationVelocity);
            prevData = encoderData;
        }
    }
    double getP(double error, double kP){
        return Math.min(1,Math.abs(error)*kP);
    }

    double getPD(double error, double dError, double kP, double kD){
        return error*kP + dError*kD;
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
    void placeAndGrab(double x1, double y1, double x2, double y2, String skyStonePosition, boolean isRedSide) {
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
        //Cross bridge
        //localizer.useEncoderOnlyToggle(true);
        directionalDrive(x1+(7 * multiplier), y1 - 5, false, 2,0);
        //Get in position to place stone
        localizer.useEncoderOnlyToggle(false);
        directionalDrive(x1, y1, true, 2,0);
        //Place stone
        if(isRedSide) {
            frontReleaseStone();
            sleep(300);
            frontCarryStone();
        }
        else{
            backReleaseStone();
            sleep(300);
            backCarryStone();
        }
        //Prepare to cross bridge
        directionalDrive(x1+(7 * multiplier), y1 - 5, false, 2,0);
        int returnDistance = 70;
        //straighteningEncoderDriveNoStop(returnDistance*multiplier, 0, 50, 1);
        if(isRedSide){
            //Cross Bridge
            //localizer.useEncoderOnlyToggle(true);
            directionalDrive(x1+7*multiplier, y2 + adjustment - 10, false, 1,0);
            frontReadyStone();
            //Pick up stone
            localizer.useEncoderOnlyToggle(false);
            directionalDrive(x2, y2 + adjustment, true, 1,0);
            frontGrabStone();
        }
        else{
            backReadyStone();
            directionalDrive(x2, y2 + adjustment, true, 1,0);
            backGrabStone();
        }
        sleep(250);
        frontCarryStone();
        backCarryStone();
        if (isRedSide) {
            //Prepare to cross bridge
            directionalDrive(SKYSTONEAutonomousConstants.stoneAwayXRed, SKYSTONEAutonomousConstants.stoneAwayY, true, 2, 0);
        } else{
            directionalDrive(SKYSTONEAutonomousConstants.stoneAwayXBlue, SKYSTONEAutonomousConstants.stoneAwayY, true, 2, 0);
        }
        returnDistance = -50;
        //straighteningEncoderDriveNoStop(returnDistance*multiplier, 0, 50, 1);
    }
    void grabFoundation(boolean isRedSide) {
        int multiplier = 1;
        if(!isRedSide){
            multiplier = -1;
        }
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        if(isRedSide){
            frontReleaseStone();
        }
        else{
            backReleaseStone();
        }


        encoderStrafeDriveInchesRight(-5,1);
        encoderTurn(90, 0.8, 2);
        //distanceEncoderDrive(38,1,0.8, 90, myRobot.frontDistance);
        runMotors(-0.4,-0.4);
        sleep(500);
        runMotors(0,0);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lDown);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rDown);
        sleep(250);
        if(isRedSide){
            //encoderStrafeDriveInchesRight(10,1);
            turn(41, 1, 0.4, 6);
            turn(0, 0, -1, 6);
            //encoderStraightDrive(-3, 1);
        }
        else{
            //encoderStrafeDriveInchesRight(-10,1);
            turn(141, 0.28,1,6);
            turn(178,-1, 0,6);
            //encoderStraightDrive(-3, 1);
        }
        //encoderStraightDrive(-12,1);
        myRobot.leftFoundationServo.setPosition(SKYSTONEConstants.lUp);
        myRobot.rightFoundationServo.setPosition(SKYSTONEConstants.rUp);
        sleep(250);
        encoderStraightDrive(7,1);
    }

    String detectSkyStonePosition(boolean isRedSide) {
        int multiplier = 1;
        double frontHue = getHue(myRobot.frontColor);
        double backHue = getHue(myRobot.backColor);
        String skyStonePosition = "Center";
        if(frontHue>70 && frontHue>backHue){
            if(isRedSide){
                skyStonePosition = "Depot";
            }
            else{
                skyStonePosition = "Bridge";
            }
        }
        else if(backHue>70 && backHue>frontHue){
            if(isRedSide){
                skyStonePosition = "Bridge";
            }
            else{
                skyStonePosition = "Depot";
            }

        }
        Log.d("Skystone: ", "FrontHue: " + frontHue + " BackHue: " + backHue);
        if(isRedSide){
            myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsOpen);
            myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbUp);
        }
        else{
            multiplier = -1;
        }
        if(skyStonePosition.equals("Center")){
            directionalDrive(SKYSTONEAutonomousConstants.stoneGrabXRed, SKYSTONEAutonomousConstants.stoneGrabY + 24 + 8,
                    true, 2, 0);
            //encoderStraightDrive(-8*multiplier, 1);
            //distanceEncoderDrive(baseDistance+8,0.5,1,0, myRobot.frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        if(skyStonePosition.equals("Bridge")){
            directionalDrive(SKYSTONEAutonomousConstants.stoneGrabXRed, SKYSTONEAutonomousConstants.stoneGrabY + 24 + 16,
                    true, 2, 0);
            //encoderStraightDrive(-16*multiplier,1);
            //distanceEncoderDrive(baseDistance+16,0.5,1,0, myRobot.frontDistance);
            //directionalDrive(SKYSTONEAutonomousConstants.fieldSize - 27, 8.0/3+18.0, true, 2,0);
        }
        Log.d("Skystone: ", " skyStonePosition: " + skyStonePosition);
        telemetry.addData("SkystonePosition", skyStonePosition);
        telemetry.update();
        return skyStonePosition;
    }

    void park(boolean isRedSide){
        double skyBridgeDistance = SKYSTONEAutonomousConstants.skyBridgeDrive;
        resetClaws();
        int power = 1;
        int angle;
        if (isRedSide){
            angle = 0;
            encoderStrafeDriveInchesRight(SKYSTONEAutonomousConstants.foundationClear, 1);
            encoderTurn(angle, 1, 2);
            /*
            runMotors(-.7,-.7);
            sleep(400);
            runMotors(0,0);
            skyBridgeDistance += 10;
            */
        } else{
            angle = 180;
            encoderStrafeDriveInchesRight(-SKYSTONEAutonomousConstants.foundationClear, 1);
            encoderTurn(angle, 1, 2);
        }
        encoderStraightDrive(skyBridgeDistance, 1);
    }
    void resetClaws(){
        myRobot.backGrabber.setPosition(SKYSTONEAutonomousConstants.bsGrab);
        myRobot.frontGrabber.setPosition(SKYSTONEAutonomousConstants.fsGrab);
        myRobot.backBase.setPosition(SKYSTONEAutonomousConstants.bbStartPosition);
        myRobot.frontBase.setPosition(SKYSTONEAutonomousConstants.fbStartPosition);
    }
}