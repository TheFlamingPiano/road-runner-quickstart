package org.firstinspires.ftc.teamcode.snappy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmData {

    LinearOpMode opmode;

    public double rotation;
    public double height;
    public double wrist;

    public long targetTime;

    public long currentTime = System.nanoTime();
    public long startTime = currentTime;


    int BaseArmCurrent;
    int IntakeArmCurrent;
    int RotationArmCurrent;//rotation

    double currentBaseArmAngle;
    double TargetBaseArmAngle;
    double currentIntakeArmAngle;
    double TargetIntakeArmAngle;
    double currentRotationAngle;

    int BaseArmNew;
    int IntakeArmNew;
    int RotationArmNew;


}
