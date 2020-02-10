package org.firstinspires.ftc.teamcode;

public class PiP {
    double rX;
    double rY;
    double oX;
    double oY;
    double eTS;
    double oTS;

    public PiP(double ratioX, double ratioY, double encoderTS, double opticalX, double opticalY, double opticalTS){
        rX = ratioX;
        rY = ratioY;
        eTS = encoderTS;
        oX = opticalX;
        oY = opticalY;
        oTS = opticalTS;
    }
}
