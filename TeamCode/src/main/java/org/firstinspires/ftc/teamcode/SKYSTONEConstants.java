package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
class SKYSTONEConstants {
    //TicksPerRotation is 42.78 on testbot
    private static double TICKS_PER_ROTATION = 103.6/3.7;
    private static double GEAR_RATIO = 20;
    private static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);
    //Foundation Constants
    static double aFoundationDistance = -28;
    static double bFoundationClear = -20;
    static double cSkybridgeClear = 20;
    static double dSkyStoneAlign = -10;
    //Skystone Constants
    static double _depotDistance = -24;
    static double _aSkyStoneDistance = -32;
    static double _bBridgeCrossDistance = -55;
    static double _cBridgeReturnDistance = 40;
    //Shared Constants
    static double shiftDistance = 8;
    static int raiseTicks = 600;
    static int lowerTicks = -100;
    static int extendSlide = -1200;
    //Skystone Tele-op Stacking Claw Rotation Servo Constants
    static double zero = 0.495; //Angled straight
    static double right90 = 0.375; //90 degrees right from straight
    static double left90 = 0.595; //90 degrees left from straight
    static double straight = 0.495; //Straight
    static double oppositeSide = 0.695; //Make it straight but on the other side or back side
    //Skystone Tele-op Stacking Grabber Claw
    static double tighten = 0.2;
    static double loosen = 0.50;
    //Skystone Left Foundation Claw
    static double lDown = 0.6; //At most .65
    static double lUp = 0.88;
    //Skystone Right Foundation Claw
    static double rDown = 0.6; //At least .5
    static double rUp = 0.25;
}
