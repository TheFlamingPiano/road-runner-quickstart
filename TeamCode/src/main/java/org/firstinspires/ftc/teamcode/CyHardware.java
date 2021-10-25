package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CyHardware {


    public DcMotor TopLeftMotor = null;
    public DcMotor TopRightMotor = null;
    public DcMotor BottomRightMotor = null;
    public DcMotor BottomLeftMotor = null;

    HardwareMap hwMap = null;

    public CyHardware() {

    }


    public void init(HardwareMap themap) {

        hwMap = themap;

        TopLeftMotor = hwMap.get(DcMotor.class, "TopLeftMotor");
        TopRightMotor = hwMap.get(DcMotor.class, "TopRightMotor");
        BottomLeftMotor = hwMap.get(DcMotor.class, "BottomLeftMotor");
        BottomRightMotor = hwMap.get(DcMotor.class, "BottomRightMotor");

        TopLeftMotor.setDirection(DcMotor.Direction.FORWARD);//REVERSE
        TopRightMotor.setDirection(DcMotor.Direction.REVERSE);
        BottomRightMotor.setDirection(DcMotor.Direction.REVERSE);
        BottomLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD); //REVERSE

        TopRightMotor.setPower(0.0);
        TopLeftMotor.setPower(0.0);
        BottomRightMotor.setPower(0.0);
        BottomLeftMotor.setPower(0.0);

       runencoders();


    }

    public void stopallmotors () {

        TopLeftMotor.setPower(0);
        TopRightMotor.setPower(0);
        BottomLeftMotor.setPower(0);
        BottomRightMotor.setPower(0);
    }

    public void resetencoders () {
        TopLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TopRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runencoders () {

        TopRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TopLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}