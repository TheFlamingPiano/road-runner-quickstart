package org.firstinspires.ftc.teamcode.snappy;

import android.telecom.Call;
import android.transition.Slide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "drive")
public class SnappyTeleOp extends LinearOpMode {

    double lastHeight;
    double lastDistance;

    SnappyHardware snappy;

    boolean dpadleft;
    boolean dpadright;

    boolean dpadleftPrevious = false;
    boolean dpadrightPrevious = false;

    public  double PivotPosition;
    public double height;
    public double distance;
    double rotation;

    double ARM_HEIGHT_VELOCITY = 450.0; //mm/s
    double ARM_DISTANCE_VELOCITY = 450.0; //mm/s
    double ARM_ROTATION_VELOCITY = 100.0; //Degrees

    InverseKinematicsSnap ik;

    boolean isRED;
    // keep a list of positions to move the arms
    MoveStep bufferedMoves;

    public SnappyTeleOp (boolean isRED){

        this.isRED = isRED;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        snappy = new SnappyHardware(hardwareMap, false, (isRED)?SnappyHardware.TeamColor.RED: SnappyHardware.TeamColor.BLUE);


//INTAKE POSITIONS, SUPER COOL SYSTEM BY THE WAY
        double OpenPosition = 0.5;
        double IntakePosition = 0.3;
        double ClosePosition = 0.1;
        double DuckPosition = 0.1;

        //PIVOT FOR INTAKE THAT JOOEY MADE
         PivotPosition = 1;
        double PivotStepSize = 0.03;
        double pivotTargetAngle = -45;

        int toggleSwitchModeOn = 1;

        double currentrotationArmAngle;

        double xFingerTouch = 1;
        double yFingerTouch = 1;

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
        double[] coordinates = ik.getPoint(currentBaseArmAngle, currentIntakeArmAndle);
        distance = coordinates[0];
        height = coordinates[1];
        telemetry.addData("Distance", distance);
        telemetry.addData("Height", height);
        telemetry.addData("Base arm angle", currentBaseArmAngle);
        telemetry.addData("Intake arm angle", currentIntakeArmAndle);
        telemetry.addData("Rotation", rotation);
        telemetry.update();

        snappy.TargetRotationAngle = snappy.RotationMotor.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ROTATION + snappy.INITIAL_ROTATION_ANGLE;


        // flag to track whether the user is moving the arms to a safe position.  This is used to reset the
        // current height and distance if the safe move is stopped before reaching the target
        boolean movingToSafePosition = false;
        bufferedMoves = null;

        // arm.setEncoders();  //set encoders
        snappy.StopArmMovement();

        snappy.StopCarousel();

        snappy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // This needs to be after wait for start
        // The robot cannot move between init and play
//        arm.Pivot.setPosition(PivotPosition);

        waitForStart();
        snappy.moveEncoderWheelUp();
        snappy.Pivot.setPosition(PivotPosition);

        // move above wait for start
        // This instance of CyrusIntakeArmHardware is redundant - merge with arm?
//        ik = new CyrusIntakeArmHardware(arm.ARM1_LENGTH, arm.ARM2_LENGTH);

//TIMES FOR ARM CONTROL *PROBS IMPORTANT*

        long currentTime = System.nanoTime(); //System.currentTimeMillis();
        long lastTime = currentTime;
        snappy.BaseArm.setPower(ARM1_POWER);
        snappy.IntakeArm.setPower(ARM2_POWER);
        snappy.RotationMotor.setPower(RotationPower);


        while (!isStopRequested()) {
            snappy.setWeightedDrivePower(    //SOME WEIRDO WHO DRIVES THE DRIVE BASE USES THIS
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            //KABLAM! HERE SETTING STUFF UP WITH CONTROLLER, THIS STUFF IS GOING ONTO THE SECOND CONTROLER, ALSO KNOWN AS THE OPERATOR CONTROLLER


            snappy.UpdateArmMovement();

            if (gamepad2.touchpad_finger_1) {
                xFingerTouch = gamepad2.touchpad_finger_1_x;
                if (xFingerTouch < -0.5) {
                    toggleSwitchModeOn = 1; //This is normal mode
                    gamepad2.rumbleBlips(1);
                } else if (xFingerTouch > 0.5) {  //This is duck mode
                    toggleSwitchModeOn = 2;
                    gamepad2.rumbleBlips(2);
                } else {
                    toggleSwitchModeOn = 3; // this is the alliance hub mode
                    gamepad2.rumbleBlips(3);
                }
            }


            //toggleSwitchModeOn = gamepad2.touchpad_finger_1 || toggleSwitchModeOn;

//             if (gamepad2.dpad_up && gamepad2.left_bumper && gamepad2.right_bumper) {
//                 toggleSwitchModeOn = false;
//             }


            if (toggleSwitchModeOn == 1) {// Shared Hub Mode
                if (gamepad2.dpad_left) { //Move to the warehouse
                    if (isRED) CallArmMove(this, 0, 50, -58, -25, 1); //0 50 -57 - 10
                    else CallArmMove(this, 0, 50, -58, -25, 1); //47
                } else if (gamepad2.dpad_right) { // Move to the shared hub
                    if (isRED) CallArmMove(this, -121, 27, -9.4, 39, 1);
                    else CallArmMove(this, 121, 27, -9.4, 39, 1);
                }
            }
            if (toggleSwitchModeOn == 2) {// Duck Mode
                if (gamepad2.dpad_left) { //Move to the alliance hub
                    if (isRED) CallArmMove(this, -12, 695, 360, 0, 1); //-70 because thats the lowest it can go,should be -99
                    else CallArmMove(this, 12, 695, 360, 0, 1);
                } else if (gamepad2.dpad_right) { // Move to the ducks
                    if (isRED) CallArmMove(this, -40, 108, 62, -70, 1);//-70 because thats the lowest it can go,should be -99
                    else CallArmMove(this, 40, 108, 62, -70, 1);
                }
            }
            if (toggleSwitchModeOn == 3) {// Alliance Hub Mode
                if (gamepad2.dpad_left) { //Move to the warehouse
                    double rotation = snappy.getCurrentRotationAngle();
                    if (isRED) {
//                        CallArmMove(this, rotation, 60, -57, 0.8, 1);
//                        bufferedMoves = new MoveStep(0, 60, -57, 0.74, 1);
                        CallArmMove(this, rotation, 300, 360, -10, 0.3);
                        bufferedMoves = new MoveStep(rotation, 60, 100, -10, 0.2);
                        bufferedMoves.next = new MoveStep(0, 60, 0, -10, 0.5);
                        bufferedMoves.next.next = new MoveStep(0, 60, -57, -25, 0.2);
                    } else { // blue
                        CallArmMove(this, rotation, 300, 360, -10, 0.3);
                        bufferedMoves = new MoveStep(rotation, 60, 100, -10, 0.2);
                        bufferedMoves.next = new MoveStep(0, 60, 0, -10, 0.5);
                        bufferedMoves.next.next = new MoveStep(0, 50, -58, -25, 0.2);
                    }
                } else if (gamepad2.dpad_right) { // Move to the alliance hub
                    double rotation = snappy.getCurrentRotationAngle();
                    if (isRED)  {
//                        CallArmMove(this, 122, 22, -22, 0.8, 1);
//                        bufferedMoves = new MoveStep(122, 659, 366, 0.1, 1);
                        CallArmMove(this, rotation, 60, 100, 59, 0.2);
                        bufferedMoves = new MoveStep(122, 60, 100, 59, 0.5);
                        bufferedMoves.next = new MoveStep(122, 300, 360, 59, .6);
                        bufferedMoves.next.next = new MoveStep(122, 659, 366, 60, 0.2);
                    } else {
                        CallArmMove(this, rotation, 60, 100, 59, 0.2);
                        bufferedMoves = new MoveStep(-122, 60, 100, 59, 0.5);
                        bufferedMoves.next = new MoveStep(-122, 300, 360, 59, 0.6);
                        bufferedMoves.next.next = new MoveStep(-122, 659, 366, 60, 0.2);
                    }
                }
            }

            // if the arm is not moving and there is a buffered move, queue it up
            if ((!snappy.ArmMoveIsActive) && (bufferedMoves != null)) {
                telemetry.addData("read arm enc", snappy.IntakeArm.getCurrentPosition());

                // see if there's another move in the sequence
                CallArmMove(this, bufferedMoves.rotation, bufferedMoves.distance, bufferedMoves.height,
                        bufferedMoves.wrist, bufferedMoves.time);
                bufferedMoves = bufferedMoves.next;
            }


            double OuttakePower = -gamepad2.left_trigger;
            double IntakePower = gamepad2.right_trigger;
            //  boolean DumpDoorPower = gamepad2.right_bumper;

            //THIS IS THE NORMAL THING
            if ( (toggleSwitchModeOn == 1 || toggleSwitchModeOn == 3) ) {
                if (IntakePower > 0.1) {
                    snappy.ClawServo.setPosition(IntakePosition);
                    snappy.IntakeServo.setPosition(1);
                 } else if (OuttakePower < -0.1 ) {
                    snappy.IntakeServo.setPosition(0);
                    snappy.ClawServo.setPosition(OpenPosition);
                } else {
                    snappy.ClawServo.setPosition(ClosePosition);
                    snappy.IntakeServo.setPosition(0.5);
                }
            } else {      //THIS IS THE DUCK MODE
                if (IntakePower > 0.1) {
                    snappy.ClawServo.setPosition(DuckPosition);
                    snappy.IntakeServo.setPosition(1);
                } else if (OuttakePower < -0.1 ) {
                    snappy.IntakeServo.setPosition(0);
                    snappy.ClawServo.setPosition(OpenPosition);
                } else {
                    snappy.ClawServo.setPosition(ClosePosition);
                    snappy.IntakeServo.setPosition(0.5);
                }
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

                boolean SlideOut = gamepad1.y;
                boolean SlideIn = gamepad1.a;

                if (SlideOut == true) {
                    snappy.CaraSlideMotor.setPower(1);
                } else {
                    if (SlideIn == true) {
                        snappy.CaraSlideMotor.setPower(-1);
                    } else {
                        snappy.CaraSlideMotor.setPower(0);
                    }
                }

                boolean ServoUp = gamepad1.dpad_up;
                boolean ServoDown = gamepad1.dpad_down;

                if (ServoDown == true) {
                    snappy.moveEncoderWheelDown();
                }
                if (ServoUp == true) {
                    snappy.moveEncoderWheelUp();
                }

                //SETTING STUFF ON THE CONTROLLER ALLL THIS IS ON THE OPERATOR CONTROLLER, TRUST, DRIVER DOES NOT WANNA DO THIS
                //  boolean RotationPowerRight = gamepad2.circle;
                // boolean RotationPowerLeft = gamepad2.square;
                // double Shoulder = gamepad2.left_stick_y;
                //double Elbow = gamepad2.right_stick_y;


                boolean pivotSnapToPosition = (gamepad2.right_trigger > 0.5) && (gamepad2.left_trigger > 0.5);
                boolean PivotTiltUp = gamepad2.y && !pivotSnapToPosition;
                boolean PivotTiltDown = gamepad2.a && !pivotSnapToPosition;


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


                currentTime = System.nanoTime(); //System.currentTimeMillis();
                double deltaTime = (currentTime - lastTime) * 1e-9;
                lastTime = currentTime;

                lastHeight = height;
                lastDistance = distance;


                // currentrotationArmAngle = snappy.RotationMotor.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ROTATION + snappy.INITIAL_ROTATION_ANGLE;

                if (gamepad2.left_bumper ) { // set safe driving position for the arm
                    height = snappy.SAFE_POSITION_HEIGHT;
                    distance = snappy.SAFE_POSITION_DISTANCE;
                    movingToSafePosition = true;
                } else if (movingToSafePosition) { // bumper is no longer pressed, but we were just moving to the safe position
                    // reset the targets using the current position of the arms
                    movingToSafePosition = false;
                    currentBaseArmAngle = snappy.BaseArm.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ARM1 + snappy.INITIAL_ARM1_ANGLE;
                    currentIntakeArmAndle = snappy.IntakeArm.getCurrentPosition() * 1.0 / snappy.ENCODER_TICKS_PER_DEGREE_ARM2 + snappy.INITIAL_ARM2_ANGLE;
                    coordinates = ik.getPoint(currentBaseArmAngle, currentIntakeArmAndle);
                    distance = coordinates[0];
                    height = coordinates[1];
                } else { // otherwise update according to the gamepad inputs
                    distance = distance + ARM_DISTANCE_VELOCITY * deltaTime * (-gamepad2.right_stick_y);
                    height = height + ARM_HEIGHT_VELOCITY * deltaTime * (-gamepad2.left_stick_y);
                }
                // limit height
                if (height > snappy.MAXIMUM_HEIGHT || height < snappy.MINIMUM_HEIGHT) {
                    height = lastHeight;
                }
                // limit distance
                if (distance < snappy.INITIAL_DISTANCE) { // no need to go behind the robot
                    distance = lastDistance;
                }
                // limit reach to almost fully extended
                if (Math.hypot(height, distance) > 0.999 * (snappy.ARM1_LENGTH + snappy.ARM2_LENGTH)) {
                    height = lastHeight;
                    distance = lastDistance;
                }
                // get the new arm angles
                double[] angles = ik.getAngles(distance, height);
                if (Double.isNaN(angles[0]) || (Double.isNaN(angles[1]))) { // something bad happened, revert to old values
                    distance = lastDistance;
                    height = lastHeight;
                    angles = ik.getAngles(distance, height);
                }
                double[] point = ik.getPoint(angles[0], angles[1]);  // should be the same as (distance, height) if everything is working correctly

                snappy.TargetARM1Angle = angles[0];
                snappy.TargetARM2Angle = angles[1];

//            if (gamepad2.dpad_down && gamepad2.dpad_up ){
//                height = 51.976;
//                distance = 18.006;
//            }

//            if (gamepad2.dpad_up) {
//                height = 366.16;
//                distance = 683.91;
//                rotation = 23.77;
//            }

//            snappy.BaseArm.setTargetPosition((int) ((angles[0] - snappy.INITIAL_ARM1_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM1));
////            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2
////                    + (angles[0] - snappy.INITIAL_ARM1_ANGLE) / snappy.GEAR_RATIO_ARM2_STAGE * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));
//            snappy.IntakeArm.setTargetPosition((int) ((angles[1] - snappy.INITIAL_ARM2_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ARM2));


                //  rotation = rotation + ARM_ROTATION_VELOCITY * deltaTime / 1000 * (gamepad2.right_stick_x);

                double velocityScaling = (gamepad2.dpad_down ? 0.25 : 1.0);

                if (gamepad2.b) {
                    snappy.TargetRotationAngle = snappy.TargetRotationAngle + ARM_ROTATION_VELOCITY * velocityScaling * deltaTime * -1;
                    //arm.RotationMotor.setPower(.25);
                } else {
                    if (gamepad2.x) {
                        snappy.TargetRotationAngle = snappy.TargetRotationAngle + ARM_ROTATION_VELOCITY * velocityScaling * deltaTime * 1;
                        //arm.RotationMotor.setPower(-0.25);
                    } else {
                        snappy.RotationMotor.setPower(1);
                    }
                }

//            if (gamepad2.dpad_down && gamepad2.cross){
//
//            }

                if (snappy.TargetRotationAngle < snappy.MINIMUM_ROTATION_ANGLE) {
                    snappy.TargetRotationAngle = snappy.MINIMUM_ROTATION_ANGLE;
                }
                if (snappy.TargetRotationAngle > snappy.MAXIMUM_ROTATION_ANGLE) {
                    snappy.TargetRotationAngle = snappy.MAXIMUM_ROTATION_ANGLE;
                }
                //THIS IS FOR THE AUTOMATIC ARM SWINGING FROM DEPOT TO SHARED HUB
                dpadleft = gamepad2.dpad_left;
                dpadright = gamepad2.dpad_right;

                dpadrightPrevious = dpadright;
                dpadleftPrevious = dpadleft;


                // arm.RotationMotor.setTargetPosition((400));
                // arm.RotationMotor.setPower(.5);

                //JUST TOOK THIS OUT
                // snappy.RotationMotor.setTargetPosition((int) ((rotation - snappy.INITIAL_ROTATION_ANGLE) * snappy.ENCODER_TICKS_PER_DEGREE_ROTATION));
                // snappy.TargetRotationAngle = snappy.ArmMoveData.rotation;



                //PIVOT WHOSH THING

                if (PivotTiltUp == true && PivotPosition <= 1.0) {
                    PivotPosition = PivotPosition + PivotStepSize;
                    snappy.Pivot.setPosition(PivotPosition);
                }
                if (PivotTiltDown == true && PivotPosition >= 0) {
                    PivotPosition = PivotPosition - PivotStepSize;
                    snappy.Pivot.setPosition(PivotPosition);
                }

                if (pivotSnapToPosition) {
                    PivotPosition = (0.5) + ((pivotTargetAngle - (angles[0] - snappy.INITIAL_ARM1_ANGLE) - (angles[1] - snappy.INITIAL_ARM2_ANGLE)) / 180);
                    if (PivotPosition > 1) {
                        PivotPosition = 1;
                    }
                    if (PivotPosition < 0) {
                        PivotPosition = 0;
                    }
                    snappy.Pivot.setPosition(PivotPosition);
                }






                telemetry.addData("RightTrigger", gamepad2.right_trigger);
                telemetry.addData("LeftTrigger", gamepad2.left_trigger);
                //  telemetry.addData("IntakeCheck", snappy.IntakeServo.getPosition());
                telemetry.addData("Height = ", height);
                telemetry.addData("Distance =", distance);
                telemetry.addData("Rotation =", snappy.TargetRotationAngle);

                telemetry.addData("PivotAngle", snappy.RequestedWristAngle); //TAKE THIS OUT!!!!! DONt LET thIS STAY IN! ON SECOND thOGUHT I CAHNGED IT SO NEVERMIND :D

                telemetry.addData("AngleRotation", (snappy.TargetRotationAngle - snappy.INITIAL_ROTATION_ANGLE));
                //telemetry.addData("RotationMotorInt",((int)(rotation * arm.ENCODER_TICKS_PER_DEGREE_ROTATION)));
                // telemetry.addData("ENCODER_COUNT_ROTO", snappy.RotationMotor.getCurrentPosition());
                telemetry.addData("ARM1_ANGLE", angles[0]);
                telemetry.addData("ARM2_ANGLE", angles[1]);
                telemetry.addData("FingerXPosition", xFingerTouch);
                telemetry.addData("Mode", toggleSwitchModeOn);
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
    void CallArmMove(LinearOpMode opmode, double rotationx, double distancex, double heightx, double wrist, double targetTime) {

        //PivotPosition = wrist;   // This is a problem since wrist is now an angle, not servo position
        PivotPosition = snappy.setWristPosition(wrist, distancex, heightx);  // this is a Hack to get the correct servo position for wrist and set global wrist variable
        height = heightx;
        distance = distancex;
        lastHeight = height;
        lastDistance = distance;

        snappy.NoneCodeBlockingArmMovement(this, rotationx, distancex, heightx, wrist, targetTime);
    }

    private class MoveStep {
        double rotation;
        double distance;
        double height;
        double wrist;
        double time;
        MoveStep next;
        public MoveStep(double rotation, double distance, double height, double wrist, double time) {
            this.rotation = rotation;
            this.distance = distance;
            this.height = height;
            this.wrist = wrist;
            this.time = time;
            this.next = null;
        }
    }
}
