package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

public class Localizer {
    private double x = 0;
    private double y = 0;

    //Kinematics Ratio with Encoder
    public static double strafeRatio = 0.9;
    public static double straightRatio = 1.1;

    private SKYSTONEClass myRobot;
    private boolean encoder = false;
    private Corner corner = Corner.LEFT_DOWN;
    ElapsedTime clock;
    ArrayList<PiP> ALPIP;

    public Localizer(SKYSTONEClass input) {
        this.myRobot = input;
        clock = new ElapsedTime();
        ALPIP = new ArrayList<PiP>();
    }
    enum Corner {
        LEFT_UP,
        RIGHT_UP,
        LEFT_DOWN,
        RIGHT_DOWN
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public void setCoordinates(int newX, int newY){
        x = newX;
        y = newY;
        updateCorner();
    }
    private void updateCorner(){
        if(x <SKYSTONEAutonomousConstants.fieldSize/2){
            if(y <SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Localizer.Corner.LEFT_DOWN;
            }
            else{
                corner = Localizer.Corner.LEFT_UP;
            }
        } else {
            if(y <SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Localizer.Corner.RIGHT_DOWN;
            }
            else{
                corner = Localizer.Corner.RIGHT_UP;
            }
        }
    }

    public Corner getCorner(){
        return corner;
    }


    public void update(RevBulkData prevData, RevBulkData encoderData){
        int lfPosition = encoderData.getMotorCurrentPosition(myRobot.lf);
        int lbPosition = encoderData.getMotorCurrentPosition(myRobot.lb);
        int rfPosition = encoderData.getMotorCurrentPosition(myRobot.rf);
        int rbPosition = encoderData.getMotorCurrentPosition(myRobot.rb);
        int lfVelocity = encoderData.getMotorVelocity(myRobot.lf);
        int lbVelocity = encoderData.getMotorVelocity(myRobot.lb);
        int rfVelocity = encoderData.getMotorVelocity(myRobot.rf);
        int rbVelocity = encoderData.getMotorVelocity(myRobot.rb);
        double newDiffEncoderX = diffEncoderX(prevData, encoderData);
        double encoderX = newDiffEncoderX + x;
        double newDiffEncoderY = newDiffEncoderY(prevData, encoderData);
        double encoderY = newDiffEncoderY + y;
        if (!encoder) {
            double newDiffOpticalX = getXRaw() - x;
            x = newDiffOpticalX + x;
            double newDiffOpticalY = getYRaw() - y;
            y = newDiffOpticalY + y;

            if (x > 250 || x < -100) {
                x = encoderX;
                Log.d("Skystone: ", "XOutOfBounds encoderX: " + x);
            }
            if (y > 250 || y < -100) {
                y = encoderY;
                Log.d("Skystone: ", "YOutOfBounds encoderY: " + y);
            }
            double t1 = clock.nanoseconds();
            PiP value1 = new PiP(newDiffOpticalX/newDiffEncoderX, newDiffOpticalY/newDiffEncoderY, t1, x, y, t1);
            if (ALPIP.size() >= 10){
                ALPIP.remove(0);
            }
            ALPIP.add(value1);
        } else {
            x = encoderX;
            y = encoderY;
        }

        Log.d("Skystone: ", "Encoder Positions: lf: " + lfPosition + " lb: " + lbPosition +
                " rf: " + rfPosition + " rb: " + rbPosition);
        Log.d("Skystone: ", "Wheel Velocities: lf: " + lfVelocity + " lb: " + lbVelocity +
                " rf: " + rfVelocity + " rb: " + rbVelocity);
        Log.d("Skystone: ", "finalPosition: x: " + x + " y: " + y + " encoderX " + encoderX +
                " encoderY " + encoderY);
        updateCorner();
    }

    private double diffEncoderX(RevBulkData prevData, RevBulkData curData){
        double multiplier = strafeRatio;
        if(corner == Localizer.Corner.RIGHT_DOWN || corner == Localizer.Corner.RIGHT_UP){
            multiplier = - strafeRatio;
        }
        return (getTotalXPositions(curData)-getTotalXPositions(prevData))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;

    }

    private double newDiffEncoderY(RevBulkData prevData, RevBulkData curData){
        double multiplier = straightRatio;
        if(corner == Localizer.Corner.RIGHT_DOWN || corner == Localizer.Corner.RIGHT_UP){
            multiplier = - straightRatio;
        }
        return (getTotalYPositions(curData) - getTotalYPositions(prevData))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;
    }

    private int getTotalXPositions(RevBulkData curData) {
        return curData.getMotorCurrentPosition(this.myRobot.lf)
                - curData.getMotorCurrentPosition(myRobot.lb)
                - curData.getMotorCurrentPosition(myRobot.rf)
                + curData.getMotorCurrentPosition(myRobot.rb);
    }

    private int getTotalYPositions(RevBulkData curData){
        return curData.getMotorCurrentPosition(myRobot.lf)
                + curData.getMotorCurrentPosition(myRobot.lb)
                + curData.getMotorCurrentPosition(myRobot.rf)
                + curData.getMotorCurrentPosition(myRobot.rb);
    }
    private double getXRaw() {
        double xRaw = 0;
        switch (corner){
            case LEFT_DOWN: case LEFT_UP:
                xRaw = myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                if(xRaw>30){
                    xRaw = 400;
                }
                break;
            case RIGHT_DOWN: case RIGHT_UP:
                xRaw = SKYSTONEAutonomousConstants.fieldSize - myRobot.leftDistance.getDistance(DistanceUnit.INCH);
                if(xRaw<SKYSTONEAutonomousConstants.fieldSize - 35){
                    xRaw = -400;
                }
                break;
        }
        return xRaw;
    }

    private double getYRaw() {
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

    public double getdY(RevBulkData bulkData){
        //Robot heading is flipped on red, so positive encoder = negative position
        int multiplier = 1;
        if(corner == Localizer.Corner.RIGHT_DOWN || corner == Localizer.Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (bulkData.getMotorVelocity(myRobot.lf)
                + bulkData.getMotorVelocity(myRobot.lb)
                + bulkData.getMotorVelocity(myRobot.rf)
                + bulkData.getMotorVelocity(myRobot.rb))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;
    }
    public double getdX(RevBulkData bulkData){
        //Robot heading is flipped on red, so positive encoder = negative position
        int multiplier = 1;
        if(corner == Localizer.Corner.RIGHT_DOWN || corner == Localizer.Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (bulkData.getMotorVelocity(myRobot.lf)
                - bulkData.getMotorVelocity(myRobot.lb)
                - bulkData.getMotorVelocity(myRobot.rf)
                + bulkData.getMotorVelocity(myRobot.rb))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier;
    }

    public void useEncoderOnlyToggle(boolean useEncoderOnly){
        if (useEncoderOnly){
            encoder = true;
        } else if (!useEncoderOnly){
            encoder = false;
        }
    }

    public void averageDiffs(){
        double xSum = 0;
        double ySum = 0;
        double size = ALPIP.size();
        for(int i = 0; i < size; i++){
            xSum = xSum + ALPIP.get(i).rX;
            ySum = ySum + ALPIP.get(i).rY;
        }
        strafeRatio = xSum/size;
        straightRatio = ySum/size;
        Log.d("Skystone:: ", "strafeRatio: " + strafeRatio + " straightRatio: " + straightRatio);
    }
}
