package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CyrusIntakeArmHardware {

    public DcMotor IntakeMotor;
    public Servo DumpDoor;
    public Servo Pivot;
    public DcMotorEx RotationMotor;
    public DcMotorEx BaseArm;
    public DcMotorEx IntakeArm;

    public CyrusIntakeArmHardware(HardwareMap hardwareMap){

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");  //Control Hub 2, Port 3
        DumpDoor = hardwareMap.get(Servo.class, "DumpDoor"); //port 1
        Pivot = hardwareMap.get(Servo.class, "Pivot");//port 2 pivot ADD THIS TO CONFIG OR ELSE EVEYTHING WILL EXPLODE!!!!

        RotationMotor = hardwareMap.get(DcMotorEx.class, "RotationMotor"); //Hub 2, Port 0
        BaseArm = hardwareMap.get(DcMotorEx.class, "BaseArm");    //Hub 2, Port 1
        IntakeArm = hardwareMap.get(DcMotorEx.class, "IntakeArm");  //Hub 2, Port 2

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        DumpDoor.setDirection(Servo.Direction.FORWARD);

       RotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       BaseArm.setDirection(DcMotorSimple.Direction.FORWARD);
       IntakeArm.setDirection(DcMotorSimple.Direction.FORWARD);


    }

        public void StopArmMovement () {

        IntakeMotor.setPower(0.0);
        DumpDoor.setPosition(0.0);
        RotationMotor.setPower(0.0);
        BaseArm.setPower(0.0);
        IntakeArm.setPower(0.0);

    }
        public void setEncoders () {
            IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BaseArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

}
