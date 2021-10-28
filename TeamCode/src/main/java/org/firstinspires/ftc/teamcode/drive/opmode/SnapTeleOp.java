package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.CyrusCarouselHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusOfficialHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class SnapTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CyrusOfficialHardware drive = new CyrusOfficialHardware(hardwareMap);
        CyrusIntakeArmHardware arm = new CyrusIntakeArmHardware(hardwareMap);
        CyrusCarouselHardware duck = new CyrusCarouselHardware(hardwareMap);

        double ClosePosition = 0.3;
        double IntakePosition = 0.45;
        double DumpPosition = 0.55;
        double CarouselPosition = 0.5;


        arm.setEncoders();  //set encoders
        arm.StopArmMovement();

        duck.Stop();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            //write here me
            double OuttakePower = -gamepad1.left_trigger;
            double IntakePower = gamepad1.right_trigger;
            Boolean DumpDoorPower = gamepad1.right_bumper;



            if (IntakePower > 0.1){

                arm.DumpDoor.setPosition(IntakePosition);

            }
            else {
                if (DumpDoorPower == Boolean.TRUE) {

                    arm.DumpDoor.setPosition(DumpPosition);

                }
                else {

                    arm.DumpDoor.setPosition(ClosePosition);

                }
            }
            if (OuttakePower < -0.1 && IntakePower < 0.1) {

                arm.IntakeMotor.setPower(OuttakePower);

            }
            else {
                arm.IntakeMotor.setPower(IntakePower);
            }


            double ClockwiseCarouselPower = gamepad2.right_trigger;
            double CounterClockwisePower = -gamepad2.left_trigger;


            if (ClockwiseCarouselPower > 0.1) {

                duck.BlueSpin();

            }
            if (CounterClockwisePower < 0.1 && ClockwiseCarouselPower < 0.1 ) {

                duck.RedSpin();

            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
