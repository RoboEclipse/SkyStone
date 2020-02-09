package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.RevBulkData;

public class Localizer {
    private double xRaw;
    private double yRaw;
    private SKYSTONEClass myRobot;
    private Corner corner;

    public Localizer(SKYSTONEClass myRobot) {
        this.myRobot = myRobot;
    }
    enum Corner {
        LEFT_UP,
        RIGHT_UP,
        LEFT_DOWN,
        RIGHT_DOWN
    }
    void update(RevBulkData prevData){
        RevBulkData encoderData = myRobot.expansionHub.getBulkInputData();
        int lfPosition = encoderData.getMotorCurrentPosition(myRobot.lf);
        int lbPosition = encoderData.getMotorCurrentPosition(myRobot.lb);
        int rfPosition = encoderData.getMotorCurrentPosition(myRobot.rf);
        int rbPosition = encoderData.getMotorCurrentPosition(myRobot.rb);
        int lfVelocity = encoderData.getMotorVelocity(myRobot.lf);
        int lbVelocity = encoderData.getMotorVelocity(myRobot.lb);
        int rfVelocity = encoderData.getMotorVelocity(myRobot.rf);
        int rbVelocity = encoderData.getMotorVelocity(myRobot.rb);
        double encoderX = backUpEncoderX(prevData, encoderData, xRaw, corner);
        double encoderY = backUpEncoderY(prevData, encoderData, yRaw, corner);
        xRaw = getXRaw(corner);
        yRaw = getYRaw(corner);
        Log.d("Skystone: ", "Encoder Positions: lf: " + lfPosition + " lb: " + lbPosition +
                " rf: " + rfPosition + " rb: " + rbPosition);
        Log.d("Skystone: ", "Wheel Velocities: lf: " + lfVelocity + " lb: " + lbVelocity +
                " rf: " + rfVelocity + " rb: " + rbVelocity);
        Log.d("Skystone: ", "xRaw: " + xRaw + " yRaw: " + yRaw + " encoderX " + encoderX +
                " encoderY " + encoderY);
        if(xRaw>250 || xRaw<-100){
            xRaw = encoderX;
            Log.d("Skystone: ", "XOutOfBounds encoderX: " + xRaw);
        }
        if(yRaw>250 || yRaw<-100){
            yRaw = encoderY;
            Log.d("Skystone: ", "YOutOfBounds encoderY: " + yRaw);
        }
    }

    private Corner getCorner(){
        if(xRaw<SKYSTONEAutonomousConstants.fieldSize/2){
            if(yRaw<SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Localizer.Corner.LEFT_DOWN;
            }
            else{
                corner = Localizer.Corner.LEFT_UP;
            }
        } else {
            if(yRaw<SKYSTONEAutonomousConstants.fieldSize/2){
                corner = Localizer.Corner.RIGHT_DOWN;
            }
            else{
                corner = Localizer.Corner.RIGHT_UP;
            }
        }
        return corner;
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
    private double getXRaw(Localizer.Corner corner) {
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

}
