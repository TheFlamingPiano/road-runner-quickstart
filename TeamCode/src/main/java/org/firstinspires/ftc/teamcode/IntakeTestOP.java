package org.firstinspires.ftc.teamcode;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "IntakeTest", group = "Tests")
//@Autonomous(name = "opmode name", group = "your group")
  // remove this line so opmode shows up in list
public class IntakeTestOP extends LinearOpMode {

    // Declare OpMode members
    public DcMotor IntakeMotor;
    public Servo DumpDoor;

    double ClosePosition = 0.3;
    double IntakePosition = 0.45;
    double DumpPosition = 0.55;

    // Main run loop. Begins when Init pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DumpDoor = hardwareMap.get(Servo.class, "DumpDoor");

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        DumpDoor.setDirection(Servo.Direction.FORWARD);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeMotor.setPower(0.0);
        DumpDoor.setPosition(0.0);    //IS IS SETPOSITION ON SERVOS???
        // Wait for Start command from Driver Station
        waitForStart();


        while (opModeIsActive()) {

            double OuttakePower = -gamepad1.left_trigger;
            double IntakePower = gamepad1.right_trigger;
            Boolean DumpDoorPower = gamepad1.right_bumper;

           // IntakeMotor.setPower(IntakePower);

            if (IntakePower > 0.1){

                DumpDoor.setPosition(IntakePosition);

            }
            else {
                if (DumpDoorPower == Boolean.TRUE) {

                    DumpDoor.setPosition(DumpPosition);

                }
                if (DumpDoorPower == Boolean.FALSE) {

                    DumpDoor.setPosition(ClosePosition);

                }
            }

            if (OuttakePower < -0.1 && IntakePower < 0.1) {

                IntakeMotor.setPower(OuttakePower);

            }
            else {
                IntakeMotor.setPower(IntakePower);
            }
        }
    }
}