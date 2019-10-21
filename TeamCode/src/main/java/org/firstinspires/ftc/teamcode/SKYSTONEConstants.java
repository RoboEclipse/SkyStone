package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEConstants {
    //TicksPerRotation is 42.78 on testbot
    private static double TICKS_PER_ROTATION = 103.6/3.7;
    private static double GEAR_RATIO = 20;
    private static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);
    //Foundation Constants
    public static double aFoundationDistance = -28;
    public static double bFoundationClear = -20;
    public static double cSkybridgeClear = 20;
    public static double dSkyStoneAlign = -10;
    //Skystone Constants
    public static double _depotDistance = -24;
    public static double _aSkyStoneDistance = 16;
    public static double _bBridgeCrossDistance = 55;
    public static double _cBridgeReturnDistance = -40;
    //Shared Constants
    public static double shiftDistance = 8;
    public static int raiseTicks = 600;
    public static int lowerTicks = -300;
    public static int extendSlide = -900;
    public static int safeSlide = -500; //Value when the rotator claw can turn without crashing into the sides
    //Skystone Tele-op Stacking Claw Rotation Servo Constants
    public static double zero = 0.495; //Angled straight
    public static double right90 = 0.375; //90 degrees right from straight
    public static double left90 = 0.595; //90 degrees left from straight
    public static double straight = 0.495; //Straight
    public static double oppositeSide = 0.695; //Make it straight but on the other side or back side
    //Skystone Tele-op Stacking Grabber Claw
    public static double tighten = 0.2;
    public static double loosen = 0.50;
    //Skystone Left Foundation Claw
    public static double lDown = 0.6; //At most .65
    public static double lUp = 0.88;
    //Skystone Right Foundation Claw
    public static double rDown = 0.6; //At least .5
    public static double rUp = 0.25;
}
