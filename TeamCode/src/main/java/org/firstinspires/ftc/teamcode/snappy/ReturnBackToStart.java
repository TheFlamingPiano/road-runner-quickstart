package org.firstinspires.ftc.teamcode.snappy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;

//@TeleOp(name = "opmode name", group = "your group")
//@Autonomous(name = "opmode name", group = "your group")
      // remove this line so opmode shows up in list
@Autonomous
public class ReturnBackToStart extends OpMode {

    // Declare OpMode members.
    CyrusIntakeArmHardware ik;
    SnappyHardware snappy;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        snappy = new SnappyHardware(hardwareMap,false, SnappyHardware.TeamColor.BLUE);
        snappy.BaseArm.setPower(1);
        snappy.IntakeArm.setPower(1);
        snappy.RotationMotor.setPower(1);
    }



    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }



    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        snappy.Pivot.setPosition(1);
        snappy.IntakeArm.setTargetPosition(0);
        snappy.BaseArm.setTargetPosition(0);
        snappy.RotationMotor.setTargetPosition(0);

    }



    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
    }



    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

    }
}
