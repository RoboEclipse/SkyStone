package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=44.4, gearing=3.7, maxRPM=1784, orientation=Rotation.CW)
@DeviceProperties(xmlTag="NestRest37CW", name="CW NeveRest 3.7 v1 Gearmotor", builtIn = true)
@DistributorInfo(distributor="AndyMark", model="am-3461")
@ModernRoboticsMotorControllerParams(P=128, I=40, D=192, ratio=57)
public interface NestRest37CW
{
}
