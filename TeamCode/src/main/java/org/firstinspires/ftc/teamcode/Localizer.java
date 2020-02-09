package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.RevBulkData;

public class Localizer {
    private double x = 0;
    private double y = 0;
    private SKYSTONEClass myRobot;
    private Corner corner = Corner.LEFT_DOWN;

    public Localizer(SKYSTONEClass input) {
        this.myRobot = input;

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
        double encoderX = backUpEncoderX(prevData, encoderData, x, corner);
        double encoderY = backUpEncoderY(prevData, encoderData, y, corner);
        x = getXRaw();
        y = getYRaw();
        Log.d("Skystone: ", "Encoder Positions: lf: " + lfPosition + " lb: " + lbPosition +
                " rf: " + rfPosition + " rb: " + rbPosition);
        Log.d("Skystone: ", "Wheel Velocities: lf: " + lfVelocity + " lb: " + lbVelocity +
                " rf: " + rfVelocity + " rb: " + rbVelocity);
        Log.d("Skystone: ", "x: " + x + " y: " + y + " encoderX " + encoderX +
                " encoderY " + encoderY);
        if(x >250 || x <-100){
            x = encoderX;
            Log.d("Skystone: ", "XOutOfBounds encoderX: " + x);
        }
        if(y >250 || y <-100){
            y = encoderY;
            Log.d("Skystone: ", "YOutOfBounds encoderY: " + y);
        }
        updateCorner();
    }

    private double backUpEncoderX(RevBulkData prevData, RevBulkData curData, double xRaw, Localizer.Corner corner){
        int multiplier = 1;
        if(corner == Localizer.Corner.RIGHT_DOWN || corner == Localizer.Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (getTotalXPositions(curData)-getTotalXPositions(prevData))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier + xRaw;

    }

    private double backUpEncoderY(RevBulkData prevData, RevBulkData curData, double yRaw, Localizer.Corner corner){
        int multiplier = 1;
        if(corner == Localizer.Corner.RIGHT_DOWN || corner == Localizer.Corner.RIGHT_UP){
            multiplier = -1;
        }
        return (getTotalYPositions(curData) - getTotalYPositions(prevData))
                /SKYSTONEConstants.TICKS_PER_INCH/4*multiplier + yRaw;
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

}
