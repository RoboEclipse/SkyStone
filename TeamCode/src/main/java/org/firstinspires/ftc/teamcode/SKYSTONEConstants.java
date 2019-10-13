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
    static double _bBridgeCrossDistance = 55;
    static double _cBridgeReturnDistance = -40;
    //Skystone Tele-op Stacking Claw Rotation Servo Constants
    static double zero = 0.5; //Angled straight
    static double right90 = 0.84; //90 degrees right from straight
    static double left90 = 0.17; //90 degrees left from straight
    //Skystone Tele-op Stacking Grabber Claw
    static double tighten = 0.5;
    static double loosen = 0.7;
}
