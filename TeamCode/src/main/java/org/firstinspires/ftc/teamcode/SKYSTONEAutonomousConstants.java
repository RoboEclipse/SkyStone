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
    public static double cFoundationClearPart1 = 40;
    public static double cFoundationClearPart2 = 20;
    public static double eSkybridge = 10;
    //Mixed Constants
    public static double mixedSkyStoneDistance1 = 5;
    public static double mixedAdjustDistance = 6;
    public static double mixedSkyStoneDistance2 = 15;
    public static double mixedBridgeCross = 50;
    //New Mixed Constants
    public static double driveToStoneDistance = 30;
    public static double pullStoneOutDistance = -5;
    public static double driveUnderBridgeDistance = 75;
    public static double hitFoundationDistance = -10;
    public static double dropOffStoneDistance = 15;
    public static double parkDistance = 40;
    public static double moveAwayFromStoneDistance = -10;
    //Double Constants
    public static double doubleSkyStoneDistance1 = 5;
    public static double doubleAdjustDistance = 8.5;
    public static double doubleBridgeCross = 47;
    public static double doubleWallDistance = 8;
    public static double doubleCenterDistance = 9;

    public static double _pickUpDistance = 27;
    public static double _aSkyStoneDistance = 18;
    public static double _bBridgeCrossDistance = 44;
    public static double _cBridgeReturnDistance = -35;
    public static double leftWallDistance = 26;
    //Shared Constants
    public static double shiftDistance = 7.5;
    public static double blueShift = 6;
    public static double extraShift = 0;
    public static int raiseTicks = 600;
    public static int lowerTicks = -300;
    public static int extendSlide = -850;

    public static int safeSlide = -450; //Value when the rotator claw can turn without crashing into the sides
    public static int stoneDiff = 3;
    //Skystone Tele-op Stacking Claw Rotation Servo Constants
    public static double right90 = 0.376; //90 degrees right from straight
    public static double left90 = 0.6; //90 degrees left from straight
    public static double straight = 0.488; //Straight
    public static double oppositeSide = 0.713; //Make it straight but on the other side or back side
    //Skystone Tele-op Stacking Grabber Claw
    public static double tighten = 0.05;
    public static double loosen = 0.7;
    public static double autoLoosen = 1;
    //Skystone Left Foundation Claw
    public static double lDown = 0.6; //At most .65
    public static double lUp = 0.25;
    //Skystone Right Foundation Claw
    public static double rDown = 0.25; //At least .5
    public static double rUp = 0.55;
    //Front right claw
    public static double frDown = 0.57;
    public static double frUp = 1;
    //Front left claw
    public static double flDown = 1;
    public static double flUP = 0.3;
    //Capstone Servo
    public static double cUp = 0.4;
    public static double cDown = 0;
    //Skystone Reset
    public static int startingElevatorHeight = 10;
}