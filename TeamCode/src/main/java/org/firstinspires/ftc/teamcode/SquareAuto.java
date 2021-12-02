package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Statement;

@Disabled
@Autonomous(name = "SquareAuto", group = "Auto")

public class SquareAuto extends LinearOpMode {

    CyHardware       robot = new CyHardware();

    static final double  COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 15;
    static final double WHEEL_DIAMETER_MILLIMETERS = 75;
    static final double COUNTS_PER_MILLIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MILLIMETERS * 3.1415);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("STATUS", "Reset Encoders");
        telemetry.update();

        robot.resetencoders();

        robot.runencoders();

        telemetry.addData("IM READY","HI");

        waitForStart();

        //CONTINUE BACK HERE AFTER THIS


       encoderDrive(DRIVE_SPEED, 341, 341);

//THIS IS WHERE YOU MAKE THINGS MOVE!!!!!!!!! >:0

    }

public void encoderDrive (double speed,
                          double leftMillimeters, double rightMillimeters)
                           {
    int newBottomLeftTarget;
    int newBottomRightTarget;
    int newTopRightTarget;
    int newTopLeftTarget;

    if (opModeIsActive()) {

        newBottomRightTarget = robot.BottomRightMotor.getCurrentPosition() + (int) (rightMillimeters * COUNTS_PER_MILLIMETERS);
        newBottomLeftTarget = robot.BottomLeftMotor.getCurrentPosition() + (int) (leftMillimeters * COUNTS_PER_MILLIMETERS);
        newTopRightTarget = robot.TopRightMotor.getCurrentPosition() + (int) (rightMillimeters * COUNTS_PER_MILLIMETERS);
        newTopLeftTarget = robot.TopLeftMotor.getCurrentPosition() + (int) (leftMillimeters * COUNTS_PER_MILLIMETERS);

        robot.BottomRightMotor.setTargetPosition(newBottomRightTarget);
        robot.BottomLeftMotor.setTargetPosition(newBottomLeftTarget);
        robot.TopRightMotor.setTargetPosition(newTopRightTarget);
        robot.TopLeftMotor.setTargetPosition(newTopLeftTarget);

        robot.BottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.TopLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.TopRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.BottomRightMotor.setPower(Math.abs(speed));
        robot.BottomLeftMotor.setPower(Math.abs(speed));
        robot.TopRightMotor.setPower(Math.abs(speed));
        robot.TopLeftMotor.setPower(Math.abs(speed));

        while (opModeIsActive() &&
            (robot.TopLeftMotor.isBusy() && robot.BottomLeftMotor.isBusy() && robot.TopRightMotor.isBusy() && robot.BottomRightMotor.isBusy())) {

        }

            robot.stopallmotors();

        robot.runencoders();


    }
}
}
