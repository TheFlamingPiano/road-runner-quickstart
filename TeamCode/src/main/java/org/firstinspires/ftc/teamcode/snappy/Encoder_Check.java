package org.firstinspires.ftc.teamcode.snappy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name = "opmode name", group = "your group")
//@Autonomous(name = "opmode name", group = "your group")
  // remove this line so opmode shows up in list
@Disabled
@TeleOp
public class Encoder_Check extends LinearOpMode {

    // Declare OpMode members.



    // Main run loop. Begins when Init pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        SnappyHardware snappy = new SnappyHardware(hardwareMap,true, SnappyHardware.TeamColor.BLUE);




        // Wait for Start command from Driver Station
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Arm 2 Encoder", snappy.IntakeArm.getCurrentPosition());
            telemetry.addData("Arm 1 Encoder", snappy.BaseArm.getCurrentPosition());
            telemetry.addData("Rotation", snappy.RotationMotor.getCurrentPosition());
            telemetry.addData("Left Encoder", snappy.leftFront.getCurrentPosition());
            telemetry.addData("Right Encoder", snappy.rightFront.getCurrentPosition());
            telemetry.addData("Front Encoder", snappy.rightRear.getCurrentPosition());
            telemetry.addData("Intake Sensor", snappy.sensorRange.getDistance(DistanceUnit.MM));
            telemetry.update();
        }


    }
}
