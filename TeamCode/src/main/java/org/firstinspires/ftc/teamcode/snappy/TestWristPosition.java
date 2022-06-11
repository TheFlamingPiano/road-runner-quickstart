package org.firstinspires.ftc.teamcode.snappy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestWristPosition extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        SnappyHardware snappy = new SnappyHardware(hardwareMap,true, SnappyHardware.TeamColor.BLUE);
        CameraSnap cam = new CameraSnap();

        cam.runOpMode(this,true, SnappyHardware.TeamColor.BLUE);

        waitForStart();

        while(opModeIsActive()){

            boolean set1 = gamepad2.x;


        }
    }
    }
