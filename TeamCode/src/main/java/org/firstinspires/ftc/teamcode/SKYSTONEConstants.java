package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class SKYSTONEConstants {
    //TicksPerRotation is 42.78 on testbot
    public static double TICKS_PER_ROTATION = 103.6/3.7;
    public static double GEAR_RATIO = 20;
    public static double TICKS_PER_WHEEL_ROTATION = TICKS_PER_ROTATION*GEAR_RATIO;
    public static double TICKS_PER_INCH = TICKS_PER_WHEEL_ROTATION/(4* Math.PI);
    //Foundation Constants
    public static double aFoundationDistance = -28;
    public static double bFoundationClear = -20;
    public static double cSkybridgeClear = 20;
    public static double dSkyStoneAlign = -10;
    //Skystone Constants
    public static double _aSkyStoneDistance = -32;
    public static double _bBridgeCrossDistance = 55;
    public static double _cBridgeReturnDistance = -40;
    //Skystone Tele-op Stacking Claw Rotation Servo Constants
    public static double zero = 0.5; //Angled straight
    public static double right90 = 2/3; //90 degrees right from straight
    public static double left90 = 1/3; //90 degrees left from straight
    //Skystone Tele-op Stacking Grabber Claw
    public static double tighten = 0.5;
    public static double loosen = 0.7;
}
