package org.firstinspires.ftc.teamcode.drive.opmode;

import android.widget.ThemedSpinnerAdapter;

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


    double height;
    double distance;
    double rotation;

    double ARM_HEIGHT_VELOCITY = 100; //mm/s
    double ARM_DISTANCE_VELOCITY = 100; //mm/s
    double ARM_ROTATION_VELOCITY = 180; //Degrees



    @Override
    public void runOpMode() throws InterruptedException {
        CyrusOfficialHardware drive = new CyrusOfficialHardware(hardwareMap);
        CyrusIntakeArmHardware arm = new CyrusIntakeArmHardware(hardwareMap);
        CyrusCarouselHardware duck = new CyrusCarouselHardware(hardwareMap);



//INTAKE POSITIONS, SUPER COOL SYSTEM BY THE WAY
        double ClosePosition = 0.3;
        double IntakePosition = 0.45;
        double DumpPosition = 0.55;

        //PIVOT FOR INTAKE THAT JOOEY MADE
        double PivotPosition = 0.5;
        double PivotStepSize = 0.01;

        //THAT ONE THING THAT SPINS THE DUCK
        double CarouselPosition = 0.5;

        //POWER VALUES FOR ROTATION ARM
        double RotationPower = 0.10;
        double RotationStop = 0.0;

        //BASE ARM VALUES
        double ShoulderPower = 0.5;
        double ElbowPower = 0.5;
        double StopPower = 0.0;

        height = 0.0;
        distance = 0.0;
        rotation = arm.INITIAL_ROTATION_ANGLE;

        arm.setEncoders();  //set encoders
        arm.StopArmMovement();

        duck.Stop();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

//TIMES FOR ARM CONTROL *PROBS IMPORTANT*

        long currentTime = System.currentTimeMillis();
        long lastTime = currentTime;


        while (!isStopRequested()) {
            drive.setWeightedDrivePower(    //SOME WEIRDO WHO DRIVES THE DRIVE BASE USES THIS
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            //KABLAM! HERE SETTING STUFF UP WITH CONTROLLER, THIS STUFF IS GOING ONTO THE SECOND CONTROLER, ALSO KNOWN AS THE OPERATOR CONTROLLER
            double OuttakePower = -gamepad2.left_trigger;
            double IntakePower = gamepad2.right_trigger;
            boolean DumpDoorPower = gamepad2.right_bumper;



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


            boolean ClockwiseCarouselPower = gamepad1.right_bumper;
            boolean CounterClockwisePower = gamepad1.left_bumper;


            if (ClockwiseCarouselPower == true) {

                duck.BlueSpin();

            }
            else {
                if (CounterClockwisePower == true) {

                    duck.RedSpin();

                } else {
                    duck.Stop();
                }
            }



            //SETTING STUFF ON THE CONTROLLER ALLL THIS IS ON THE OPERATOR CONTROLLER, TRUST, DRIVER DOES NOT WANNA DO THIS
          //  boolean RotationPowerRight = gamepad2.circle;
           // boolean RotationPowerLeft = gamepad2.square;
           // double Shoulder = gamepad2.left_stick_y;
            //double Elbow = gamepad2.right_stick_y;
            //boolean PivotTiltUp = gamepad2.triangle;
           // boolean PivotTiltDown = gamepad2.x;




//GETTING KATELYN'S ARM THING TO ROTATE


//if (RotationPowerRight == true) {
//
//    arm.RotationMotor.setPower(-RotationPower);
//            }
//else {
//    if (RotationPowerLeft == true) {
//
//        arm.RotationMotor.setPower(RotationPower);
//                }
//    else {
//        arm.RotationMotor.setPower(RotationStop);
//                }
//            }
//
//
////SO THIS IS WHERE KATELYNS ARM THINGY GOES WHOOOOOSH!!! IT MOVES IT UP AND DOWN (THE BASE OF THE ARM)
//
//            if (Shoulder > 0.1) {
//
//                arm.BaseArm.setPower(-ShoulderPower);
//
//            }
//            else {
//                if (Shoulder < -0.1) {
//                    arm.BaseArm.setPower(ShoulderPower);
//                }
//                else {
//                    arm.BaseArm.setPower(StopPower);
//                }
//            }
//
//            //THIS IS THE PART THAT MOVES THE INTAKE ARM (THE ONE THE INTAKE ATTACHES TOO, WHICH BY THE WAY HAVE I TOLD YOU IS A WONDERFUL SYSTEM)
//
//            if (Elbow > 0.1) {
//
//                arm.IntakeArm.setPower(ElbowPower);
//
//            }
//            else {
//                if (Elbow < -0.1) {
//                    arm.IntakeArm.setPower(-ElbowPower);
//                }
//                else {
//                    arm.IntakeArm.setPower(StopPower);
//                }
//            }

            //PIVOT WHOSH THING

//            if (PivotTiltUp == true && PivotPosition <= 1.0) {
//                PivotPosition = PivotPosition + PivotStepSize;
//                arm.Pivot.setPosition(PivotPosition);
//            }
//            if (PivotTiltDown == true && PivotPosition >= -1.0) {
//                PivotPosition = PivotPosition - PivotStepSize;
//                arm.Pivot.setPosition(PivotPosition);
//            }

         //   double ChangeHeight = -gamepad2.left_stick_y;



            currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            double lastHeight = height;

            height = height + ARM_HEIGHT_VELOCITY * deltaTime/1000 * (-gamepad2.left_stick_y);

            double lastDistance = distance;
            distance = distance + ARM_DISTANCE_VELOCITY * deltaTime/1000 * (-gamepad2.right_stick_y);

            if (height > arm.MAXIMUM_HEIGHT || height < arm.MINIMUM_HEIGHT || Math.hypot(height, distance) > arm.ARM1 + arm.ARM2 ) {
                height = lastHeight;
                distance = lastDistance;
            }


          //  rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime/1000 * (gamepad2.right_stick_x);

            if (gamepad2.circle) {
                rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime/1000 * -1;
            }

            if (gamepad2.square) {
                rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime/1000 * 1;
            }

            if (rotation < arm.MINIMUM_ROTATION_ANGLE) {
                rotation = arm.MINIMUM_ROTATION_ANGLE;
            }
            if (rotation > arm.MAXIMUM_ROTATION_ANGLE) {
                rotation = arm.MAXIMUM_ROTATION_ANGLE;
            }



            telemetry.addData("Height = ",height);
            telemetry.addData("Distance =", distance);
            telemetry.addData("Rotation =", rotation);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
