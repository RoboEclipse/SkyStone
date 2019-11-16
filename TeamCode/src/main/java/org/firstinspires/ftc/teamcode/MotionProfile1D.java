package org.firstinspires.ftc.teamcode;

public class MotionProfile1D {

    double totalDistance;
    double maxVelocity;
    double maxAcceleration;

    double currentDistance;
    double currentVelocity;
    double currentAcceleration;

    double oldTime;
    double currentTime;

    double endVelocity;


    public MotionProfile1D(double distance, double maxVelocity, double maxAcceleration, double startVelocity, double endVelocity){

        this.totalDistance=distance;
        this.maxVelocity=maxVelocity;
        this.maxAcceleration=maxAcceleration;

        this.currentDistance=0;
        this.currentVelocity=startVelocity;
        this.currentAcceleration=0;

        this.oldTime=0;
        this.currentTime=0;

        this.endVelocity=endVelocity;

    }

    public double rateLimiter (double want, double current, double maxRate){

        double maxChange = maxRate * (this.currentTime - this.oldTime);
        return Math.max(Math.min(want-current, maxChange), -maxChange);

    }

    public double calcSlowDown (){

        double distanceToSlowDown = (Math.pow(this.endVelocity, 2) - Math.pow(this.maxVelocity, 2))/(-2 * this.maxAcceleration);
        return Math.sqrt(Math.max(0, Math.pow(this.maxVelocity, 2) - 2 * this.maxAcceleration * (this.currentDistance - this.totalDistance + distanceToSlowDown)));

    }

    //maybe rename to "update"?
    public void loop(double newTime){
        this.oldTime=this.currentTime;
        this.currentTime=newTime;


        this.currentDistance += this.currentVelocity * (this.currentTime-this.oldTime);

        double oldVelocity = this.currentVelocity;
        this.currentVelocity = Math.min(this.rateLimiter(this.maxVelocity, this.currentVelocity, this.maxAcceleration) + this.currentVelocity, this.calcSlowDown());

        this.currentAcceleration = (this.currentVelocity-oldVelocity) / (this.currentTime-this.oldTime+0.0000001);
    }



}
