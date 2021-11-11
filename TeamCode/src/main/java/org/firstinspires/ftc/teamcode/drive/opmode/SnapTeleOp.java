package org.firstinspires.ftc.teamcode.drive.opmode;

import android.widget.ThemedSpinnerAdapter;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.InverseKinematics;
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

    double ARM_HEIGHT_VELOCITY = 450; //mm/s
    double ARM_DISTANCE_VELOCITY = 450; //mm/s
    double ARM_ROTATION_VELOCITY = 20; //Degrees

 CyrusIntakeArmHardware ik;


    @Override
    public void runOpMode() throws InterruptedException {
        CyrusOfficialHardware drive = new CyrusOfficialHardware(hardwareMap);
        CyrusIntakeArmHardware arm = new CyrusIntakeArmHardware(hardwareMap, false);
        CyrusCarouselHardware duck = new CyrusCarouselHardware(hardwareMap);



//INTAKE POSITIONS, SUPER COOL SYSTEM BY THE WAY
        double ClosePosition = 0.3;
        double IntakePosition = 0.45;
        double DumpPosition = 0.55;

        //PIVOT FOR INTAKE THAT JOOEY MADE
        double PivotPosition = 1;
        double PivotStepSize = 0.01;

        //THAT ONE THING THAT SPINS THE DUCK
        double CarouselPosition = 0.5;

        //POWER VALUES FOR ROTATION ARM
        double RotationPower = 1;
        double RotationStop = 0.0;

        //BASE ARM VALUES
        double ARM1_POWER = 1;
        double ARM2_POWER = 1;
        double StopPower = 0.0;

        // moved before the initialization of height and distance to back calculate the initial position
        ik = new CyrusIntakeArmHardware(arm.ARM1_LENGTH, arm.ARM2_LENGTH);
//        height = 0.0; //millmeters
//        distance = -55;//millimeters
        // Calculate the current height and distance based on the current angles of the arms
        // These should be moved to the hardware class
        double currentBaseArmAngle = arm.BaseArm.getCurrentPosition() * 1.0 / arm.ENCODER_TICKS_PER_DEGREE_ARM1 + arm.INITIAL_ARM1_ANGLE;
        double currentIntakeArmAndle = arm.IntakeArm.getCurrentPosition() * 1.0 / arm.ENCODER_TICKS_PER_DEGREE_ARM2 + arm.INITIAL_ARM2_ANGLE;
        double coordinates[] = ik.getPoint(currentBaseArmAngle, currentIntakeArmAndle);
        distance = coordinates[0];
        height = coordinates[1];
telemetry.addData("Distance", distance);
telemetry.addData("Height", height);
telemetry.update();
        rotation = arm.INITIAL_ROTATION_ANGLE;

       // arm.setEncoders();  //set encoders
        arm.StopArmMovement();

        duck.Stop();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // This needs to be after wait for start
        // The robot cannot move between init and play
//        arm.Pivot.setPosition(PivotPosition);

        waitForStart();
        arm.Pivot.setPosition(PivotPosition);

        // move above wait for start
        // This instance of CyrusIntakeArmHardware is redundant - merge with arm?
//        ik = new CyrusIntakeArmHardware(arm.ARM1_LENGTH, arm.ARM2_LENGTH);

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
            boolean PivotTiltUp = gamepad2.y;
            boolean PivotTiltDown = gamepad2.a;

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

            if (PivotTiltUp == true && PivotPosition <= 1.0) {
                PivotPosition = PivotPosition + PivotStepSize;
                arm.Pivot.setPosition(PivotPosition);
            }
            if (PivotTiltDown == true && PivotPosition >= 0) {
                PivotPosition = PivotPosition - PivotStepSize;
                arm.Pivot.setPosition(PivotPosition);
            }

         //   double ChangeHeight = -gamepad2.left_stick_y;



            currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            double lastHeight = height;

            height = height + ARM_HEIGHT_VELOCITY * deltaTime/1000 * (-gamepad2.left_stick_y);

            double lastDistance = distance;
            distance = distance + ARM_DISTANCE_VELOCITY * deltaTime/1000 * (-gamepad2.right_stick_y);







          //  rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime/1000 * (gamepad2.right_stick_x);

            if (gamepad2.x) {
               // rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime/1000 * -1;
                arm.RotationMotor.setPower(.25);
            }
            else {
                if (gamepad2.b) {
                    //rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime/1000 * 1;
                    arm.RotationMotor.setPower(-0.25);
                } else {
                    arm.RotationMotor.setPower(0);
                }
            }
            if (rotation < arm.MINIMUM_ROTATION_ANGLE) {
                rotation = arm.MINIMUM_ROTATION_ANGLE;
            }
            if (rotation > arm.MAXIMUM_ROTATION_ANGLE) {
                rotation = arm.MAXIMUM_ROTATION_ANGLE;
            }


//            if (gamepad2.dpad_down && gamepad2.dpad_up ){
//                height = 51.976;
//                distance = 18.006;
//            }



           // arm.RotationMotor.setTargetPosition((400));
           // arm.RotationMotor.setPower(.5);
           // arm.RotationMotor.setTargetPosition((int)(rotation * arm.ENCODER_TICKS_PER_DEGREE_ROTATION));
            //arm.RotationMotor.setPower(RotationPower);

            double angles[] = ik.getAngles(distance, height);
            double point[] = ik.getPoint(angles[0], angles[1]);  // should be the same as (distance, height) if everything is working correctly

            if (gamepad2.left_bumper) { // set safe driving position for the arm
                angles = ik.getAngles(arm.SAFE_POSITION_DISTANCE, arm.SAFE_POSITION_HEIGHT);
                // set distance and height to the current position so if the bumpers are released, the arm stops
                // this should be a method in the hardware class
                currentBaseArmAngle = arm.BaseArm.getCurrentPosition() * 1.0 / arm.ENCODER_TICKS_PER_DEGREE_ARM1 + arm.INITIAL_ARM1_ANGLE;
                currentIntakeArmAndle = arm.IntakeArm.getCurrentPosition() * 1.0 / arm.ENCODER_TICKS_PER_DEGREE_ARM2 + arm.INITIAL_ARM2_ANGLE;
                coordinates = ik.getPoint(currentBaseArmAngle, currentIntakeArmAndle);
                distance = coordinates[0];
                height = coordinates[1];

            }

            if (height > arm.MAXIMUM_HEIGHT || height < arm.MINIMUM_HEIGHT || Math.hypot(height, distance) > arm.ARM1_LENGTH + arm.ARM2_LENGTH ||
                    (Double.isNaN(angles[0]) || (Double.isNaN(angles[1])))) {
                height = lastHeight;
                distance = lastDistance;
            }

            arm.BaseArm.setTargetPosition((int) ((angles[0] - arm.INITIAL_ARM1_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM1));
            //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
            arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));

            arm.BaseArm.setPower(ARM1_POWER);
            arm.IntakeArm.setPower(ARM2_POWER);

            telemetry.addData("Height = ",height);
            telemetry.addData("Distance =", distance);
            telemetry.addData("Rotation =", rotation);
            telemetry.addData("PivotPosition",PivotPosition);
            //telemetry.addData("RotationMotorInt",((int)(rotation * arm.ENCODER_TICKS_PER_DEGREE_ROTATION)));
            telemetry.addData("ENCODER_COUNT_ROTO", arm.ENCODER_TICKS_PER_DEGREE_ROTATION);
            telemetry.addData("ARM1_ANGLE",angles[0]);
            telemetry.addData("ARM2_ANGLE",angles[1]);
            //telemetry.addData("POINT",point[0]);
            //telemetry.addData("POINT2",point[1]);
            //telemetry.addData("ARM2_INITIAL",((int) (angles[1] - arm.INITIAL_ARM2_ANGLE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
           // telemetry.addData("ENCODER_TICS_ARM2",arm.ENCODER_TICKS_PER_DEGREE_ARM2);
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
