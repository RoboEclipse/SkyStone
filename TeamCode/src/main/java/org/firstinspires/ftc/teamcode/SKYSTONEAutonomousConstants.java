package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEAutonomousConstants {

    //TicksPerRotation is 42.78 on testbot
    private static double TICKS_PER_ROTATION = 103.6/3.7;
    private static double GEAR_RATIO = 20;
    private static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);

    //Speeds
    static double maxSpeed = 57;

    //Distances:

    //Foundation Constants
    public static double aFoundationAim = -10;
    public static double bFoundationDistance = -35;
    public static double cFoundationTurn = 105;
    public static double dWallStrafe = 23;
    public static double eSkybridge1 = 20;
    public static double eSkybridge2 = 20;
    public static double cFoundationClearPart1 = 40;
    public static double cFoundationClearPart2 = 20;
    public static double eSkybridge = 10;
    //New Mixed Constants
    public static double driveToStoneDistance = 30;
    public static double pullStoneOutDistance = -5;
    public static double driveUnderBridgeDistance = 75;
    public static double hitFoundationDistance = -10;
    public static double dropOffStoneDistance = 15;
    public static double parkDistance = 40;
    public static double moveAwayFromStoneDistance = -10;
    public static double foundationAlign = 24;
    public static double foundationDistance = -13;
    public static double skystoneClear = 12;
    public static int frontClawsWaitLength = 300;
    public static double fieldSize = 144;
    //Double Constants
    public static double doubleSkyStoneDistance1 = 5;
    public static double doubleAdjustDistance = 9.5;
    public static double doubleBridgeCross = 47;
    public static double doubleWallDistance = 8;
    public static double doubleCenterDistance = 9;
    //Shared Constants
    public static double shiftDistance = 7.5;
    //PID loop constants, in inches
    public static double reducePowerDistance = 30;
    public static double flooringPower = 0.2;

    //Front Side base servo
    public static double fbReady = 0.36;
    public static double fbDown = 0.28;
    public static double fbUp = 0.66;
    //Front Side Autonomous Claw
    public static double fsReady = 0.38; //Needs to be updated
    public static double fsGrab = 0.45; //Needs to be updated
    //Front Side base servo
    public static double bbReady = 0.86;
    public static double bbDown = 0.91;
    public static double bbUp = 0.6;
    //Front Side Autonomous Claw
    public static double bsReady = 0; //Not right
    public static double bsGrab = 0; //Not right
    //Pure pursuit constants
    public static double kR = 1.0/25;
    public static double kD = 0.004;
}
