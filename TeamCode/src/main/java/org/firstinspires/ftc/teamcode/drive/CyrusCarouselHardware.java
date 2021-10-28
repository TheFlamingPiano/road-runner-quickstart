package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

public class CyrusCarouselHardware {

    public Servo CarsouselServo;

    public CyrusCarouselHardware(HardwareMap hardwareMap) {

        CarsouselServo = hardwareMap.get(Servo.class, "carousel"); //port 0

        CarsouselServo.setDirection(Servo.Direction.FORWARD);


    }

    public void Stop (){

        CarsouselServo.setPosition(0.5);
    }

    public void BlueSpin () {

        CarsouselServo.setPosition(1.0);
    }

    public void RedSpin () {

        CarsouselServo.setPosition(0.0);

    }

}