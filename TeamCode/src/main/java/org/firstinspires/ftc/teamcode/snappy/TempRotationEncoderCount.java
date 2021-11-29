package org.firstinspires.ftc.teamcode.snappy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
    public class TempRotationEncoderCount extends LinearOpMode {

        // Declare OpMode members.


        // Main run loop. Begins when Init pressed.
        @Override
        public void runOpMode() throws InterruptedException {

            SnappyHardware snappy = new SnappyHardware(hardwareMap,false);



            // Wait for Start command from Driver Station
            waitForStart();
while (opModeIsActive()){
    telemetry.addData("Rotation", snappy.RotationMotor.getCurrentPosition());
    telemetry.update();
            }

        }
    }

