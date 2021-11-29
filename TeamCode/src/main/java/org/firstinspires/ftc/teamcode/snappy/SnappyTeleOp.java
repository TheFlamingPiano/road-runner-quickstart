package org.firstinspires.ftc.teamcode.snappy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.InverseKinematics;
import org.firstinspires.ftc.teamcode.drive.CyrusCarouselHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusIntakeArmHardware;
import org.firstinspires.ftc.teamcode.drive.CyrusOfficialHardware;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class SnappyTeleOp extends LinearOpMode {


    double height;
    double distance;
    double rotation;

    double ARM_HEIGHT_VELOCITY = 450; //mm/s
    double ARM_DISTANCE_VELOCITY = 450; //mm/s
    double ARM_ROTATION_VELOCITY = 60; //Degrees

    InverseKinematicsSnap ik;


    @Override
    public void runOpMode() throws InterruptedException {
        SnappyHardware snappy = new SnappyHardware(hardwareMap, false);


//INTAKE POSITIONS, SUPER COOL SYSTEM BY THE WAY
        double ClosePosition = 0.3;
        double IntakePosition = 0.45;
        double DumpPosition = 0.55;

        //PIVOT FOR INTAKE THAT JOOEY MADE
        double PivotPosition = 1;
        double PivotStepSize = 0.03;
        double pivotTargetAngle = -90;


        //THAT ONE THING THAT SPINS THE DUCK
        double CarouselPosition = 0.5;

        //POWER VALUES FOR ROTATION ARM
        double RotationPower = 1.0;
        double RotationStop = 0.0;

        //BASE ARM VALUES
        double ARM1_POWER = 1;
        double ARM2_POWER = 1;
        double StopPower = 0.0;

        // moved before the initialization of height and distance to back calculate the initial position
        ik = new InverseKinematicsSnap(snappy.ARM1_LENGTH, snappy.ARM2_LENGTH);
//        height = 0.0; //millmeters
//        distance = -55;//millimeters
        // Calculate the current height and distance based on the current angles of the arms
        // These should be moved to the hardware class
        double currentBaseArmAngle = snappy.BaseArm.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ARM1 + snappy.INITIAL_ARM1_ANGLE;
        double currentIntakeArmAndle = snappy.IntakeArm.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ARM2 + snappy.INITIAL_ARM2_ANGLE;
        double coordinates[] = ik.getPoint(currentBaseArmAngle, currentIntakeArmAndle);
        distance = coordinates[0];
        height = coordinates[1];
        telemetry.addData("Distance", distance);
        telemetry.addData("Height", height);
        telemetry.addData("Rotation", rotation);
        telemetry.update();
        rotation = snappy.INITIAL_ROTATION_ANGLE;

        // arm.setEncoders();  //set encoders
        snappy.StopArmMovement();

        snappy.StopCarousel();

        snappy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // This needs to be after wait for start
        // The robot cannot move between init and play
//        arm.Pivot.setPosition(PivotPosition);

        waitForStart();
        snappy.Pivot.setPosition(PivotPosition);

        // move above wait for start
        // This instance of CyrusIntakeArmHardware is redundant - merge with arm?
//        ik = new CyrusIntakeArmHardware(arm.ARM1_LENGTH, arm.ARM2_LENGTH);

//TIMES FOR ARM CONTROL *PROBS IMPORTANT*

        long currentTime = System.currentTimeMillis();
        long lastTime = currentTime;


        while (!isStopRequested()) {
            snappy.setWeightedDrivePower(    //SOME WEIRDO WHO DRIVES THE DRIVE BASE USES THIS
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


            if (IntakePower > 0.1) {

                snappy.DumpDoor.setPosition(IntakePosition);

            } else {
                if (DumpDoorPower == Boolean.TRUE) {

                    snappy.DumpDoor.setPosition(DumpPosition);

                } else {

                    snappy.DumpDoor.setPosition(ClosePosition);

                }
            }
            if (OuttakePower < -0.1 && IntakePower < 0.1) {

                snappy.IntakeMotor.setPower(OuttakePower);

            } else {
                snappy.IntakeMotor.setPower(IntakePower);
            }


            boolean ClockwiseCarouselPower = gamepad1.right_bumper;
            boolean CounterClockwisePower = gamepad1.left_bumper;


            if (ClockwiseCarouselPower == true) {

                snappy.BlueSpin();

            } else {
                if (CounterClockwisePower == true) {

                    snappy.RedSpin();

                } else {
                    snappy.StopCarousel();
                }
            }


            //SETTING STUFF ON THE CONTROLLER ALLL THIS IS ON THE OPERATOR CONTROLLER, TRUST, DRIVER DOES NOT WANNA DO THIS
            //  boolean RotationPowerRight = gamepad2.circle;
            // boolean RotationPowerLeft = gamepad2.square;
            // double Shoulder = gamepad2.left_stick_y;
            //double Elbow = gamepad2.right_stick_y;

           // boolean pivotSnapToPosition = (gamepad2.right_trigger > 0.5) && (gamepad2.left_trigger > 0.5);
            boolean PivotTiltUp = gamepad2.y ;//&& !pivotSnapToPosition;
            boolean PivotTiltDown = gamepad2.a ;//&& !pivotSnapToPosition;


//            if (gamepad2.y) {
//                arm.Pivot.setPosition(1);
//            }
//            else{
//                if (gamepad2.b) {
//                    arm.Pivot.setPosition(0);
//                }
//                else {
//                    arm.Pivot.setPosition(.5);
//                }
//            }

//GETTING KATELYN'S ARM THING TO ROTATE


            //PIVOT WHOSH THING

            if (PivotTiltUp == true && PivotPosition <= 1.0) {
                PivotPosition = PivotPosition + PivotStepSize;
                snappy.Pivot.setPosition(PivotPosition);
            }
            if (PivotTiltDown == true && PivotPosition >= 0) {
                PivotPosition = PivotPosition - PivotStepSize;
                snappy.Pivot.setPosition(PivotPosition);
            }


            currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            double lastHeight = height;

            height = height + ARM_HEIGHT_VELOCITY * deltaTime / 1000 * (-gamepad2.left_stick_y);

            double lastDistance = distance;
            distance = distance + ARM_DISTANCE_VELOCITY * deltaTime / 1000 * (-gamepad2.right_stick_y);

          //  rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime / 1000 * (gamepad2.right_stick_x);

            double velocityScaling = (gamepad2.dpad_down? 0.25:1.0);

            if (gamepad2.x) {
                rotation = rotation + ARM_ROTATION_VELOCITY * velocityScaling * deltaTime/1000 * -1;
                //arm.RotationMotor.setPower(.25);
            }
            else {
                if (gamepad2.b) {
                    rotation = rotation + ARM_ROTATION_VELOCITY * velocityScaling * deltaTime/1000 * 1;
                    //arm.RotationMotor.setPower(-0.25);
                } else {
                    snappy.RotationMotor.setPower(0);
                }
             }


            if (rotation < snappy.MINIMUM_ROTATION_ANGLE) {
                rotation = snappy.MINIMUM_ROTATION_ANGLE;
            }
            if (rotation > snappy.MAXIMUM_ROTATION_ANGLE) {
                rotation = snappy.MAXIMUM_ROTATION_ANGLE;
            }
            //THIS IS FOR THE AUTOMATIC ARM SWINGING FROM DEPOT TO SHARED HUB
            if (gamepad2.dpad_left) {
                rotation = 10;
            }
            if (gamepad2.dpad_right) {
                rotation = 120;
            }

//            if (gamepad2.dpad_down && gamepad2.dpad_up ){
//                height = 51.976;
//                distance = 18.006;
//            }

//            if (gamepad2.dpad_up) {
//                height = 366.16;
//                distance = 683.91;
//                rotation = 23.77;
//            }


            // arm.RotationMotor.setTargetPosition((400));
            // arm.RotationMotor.setPower(.5);
            snappy.RotationMotor.setTargetPosition((int) ((rotation - snappy.INITIAL_ROTATION_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ROTATION));
            snappy.RotationMotor.setPower(RotationPower);


            double angles[] = ik.getAngles(distance, height);
            double point[] = ik.getPoint(angles[0], angles[1]);  // should be the same as (distance, height) if everything is working correctly

//            PivotPosition= (0.5)+((pivotTargetAngle - (angles[0]- snappy.INITIAL_ARM1_ANGLE) - (angles[1]- snappy.INITIAL_ARM2_ANGLE))/180);
//
//            if (PivotPosition > 1) {
//                PivotPosition = 1;
//            }
//            if (PivotPosition < 0) {
//                PivotPosition = 0;
//            }

            if (gamepad2.left_bumper) { // set safe driving position for the arm
                angles = ik.getAngles(snappy.SAFE_POSITION_DISTANCE, snappy.SAFE_POSITION_HEIGHT);
                // set distance and height to the current position so if the bumpers are released, the arm stops
                // this should be a method in the hardware class
                currentBaseArmAngle = snappy.BaseArm.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ARM1 + snappy.INITIAL_ARM1_ANGLE;
                currentIntakeArmAndle = snappy.IntakeArm.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ARM2 + snappy.INITIAL_ARM2_ANGLE;
                coordinates = ik.getPoint(currentBaseArmAngle, currentIntakeArmAndle);
                distance = coordinates[0];
                height = coordinates[1];

            }

            if (height > snappy.MAXIMUM_HEIGHT || height < snappy.MINIMUM_HEIGHT || Math.hypot(height, distance) > snappy.ARM1_LENGTH + snappy.ARM2_LENGTH ||
                    (Double.isNaN(angles[0]) || (Double.isNaN(angles[1])))) {
                height = lastHeight;
                distance = lastDistance;
            }

            snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));

            snappy.BaseArm.setPower(ARM1_POWER);
            snappy.IntakeArm.setPower(ARM2_POWER);

            telemetry.addData("Height = ", height);
            telemetry.addData("Distance =", distance);
            telemetry.addData("Rotation =", rotation);
            telemetry.addData("PivotPosition", PivotPosition);
            telemetry.addData("AngleRotation", (rotation - snappy.INITIAL_ROTATION_ANGLE));
            //telemetry.addData("RotationMotorInt",((int)(rotation * arm.ENCODER_TICKS_PER_DEGREE_ROTATION)));
            telemetry.addData("ENCODER_COUNT_ROTO", snappy.RotationMotor.getCurrentPosition());
            telemetry.addData("ARM1_ANGLE", angles[0]);
            telemetry.addData("ARM2_ANGLE", angles[1]);
            //telemetry.addData("POINT",point[0]);
            //telemetry.addData("POINT2",point[1]);
            //telemetry.addData("ARM2_INITIAL",((int) (angles[1] - arm.INITIAL_ARM2_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            // telemetry.addData("ENCODER_TICS_ARM2",arm.ENCODER_TICKS_PER_DEGREE_ARM2);
            snappy.update();

            Pose2d poseEstimate = snappy.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
