package org.firstinspires.ftc.teamcode.snappy;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SnappyHardware extends MecanumDrive {

    //Carousel thingy
    public Servo CarsouselServo;

    //ARM HARDWARE
    public DcMotor IntakeMotor;
    public Servo DumpDoor;
    public Servo Pivot;
    public DcMotorEx RotationMotor;
    public DcMotorEx BaseArm;
    public DcMotorEx IntakeArm;
    public final double MINIMUM_ROTATION_ANGLE = -180.0; //degrees //-135
    public final double MAXIMUM_ROTATION_ANGLE = 180.0; //degrees
    public final double INITIAL_ROTATION_ANGLE = -45.0; //degrees
    public final double INITIAL_ARM1_ANGLE = 177.18; //180;//216;//degree
    public final double INITIAL_ARM2_ANGLE = -176.2; //-180;//degree

    public double ENCODER_TICKS_PER_DEGREE_MOTOR = 28.0 / 360.0;
    public double GEARBOX_RATIO_ROTATION_MOTOR = 1.0;
    public double GEARBOX_RATIO_ARM1_MOTOR = (46.0 / 17.0 + 1) * (46.0 / 17.0 + 1);
    public double GEARBOX_RATIO_ARM2_MOTOR = (46.0 / 11.0 + 1);
    public double GEAR_RATIO_ROTATION_STAGE = (140.0 / 16.0)*(80.0/12);
    public double GEAR_RATIO_ARM1_STAGE = 19;
    public double GEAR_RATIO_ARM2_STAGE = 31;
//ang1+ang2secondbar + 90
    //ETPD CALCULATIONS
    public final double ENCODER_TICKS_PER_DEGREE_ROTATION = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ROTATION_MOTOR * GEAR_RATIO_ROTATION_STAGE;
    public final double ENCODER_TICKS_PER_DEGREE_ARM1 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM1_MOTOR * GEAR_RATIO_ARM1_STAGE;
    public final double ENCODER_TICKS_PER_DEGREE_ARM2 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM2_MOTOR * GEAR_RATIO_ARM2_STAGE;

    //ARM LENGTH
    public final double ARM1_LENGTH = 447.5; //460.0; //millimeters
    public final double ARM2_LENGTH = 469;//476.00;//405.0; //millimeters

    public final double INITIAL_HEIGHT = ARM1_LENGTH * Math.sin(Math.toRadians(INITIAL_ARM1_ANGLE)) + ARM2_LENGTH * Math.sin(Math.toRadians(INITIAL_ARM1_ANGLE + INITIAL_ARM2_ANGLE));
    public final double INITIAL_DISTANCE = ARM1_LENGTH * Math.cos(Math.toRadians(INITIAL_ARM1_ANGLE)) + ARM2_LENGTH * Math.cos(Math.toRadians(INITIAL_ARM1_ANGLE + INITIAL_ARM2_ANGLE));

    public final double SAFE_POSITION_DISTANCE = INITIAL_DISTANCE; //-70;
    public final double SAFE_POSITION_HEIGHT = INITIAL_HEIGHT; //20;
    public final double SAFE_ROTATION_ANGLE = -35; // need to test

    //THIS IS CODE FOR ARM THAT WAS ORIGINALLY IN TELE OP (HEIGHT, DISTANCE, ROTATION STUFF)
    double height;
    double distance;
    double rotation;

    double lastHeight;
    double lastDistance;

    double ARM_HEIGHT_VELOCITY = 450; //mm/s
    double ARM_DISTANCE_VELOCITY = 450; //mm/s
    double ARM_ROTATION_VELOCITY = 20; //Degrees

    InverseKinematicsSnap ik;

    double RotationPower = 1;

    double currentBaseArmAngle;
    double currentIntakeArmAndle;

    double ARM1_POWER = 1;
    double ARM2_POWER = 1;


    double ClosePosition = 0.3;
    double IntakePosition = 0.45;
    double DumpPosition = 0.55;
    //VELOCITIES
    //public final double ROTATION_VELOCITY = 50; //degrees per second
//    public final double HEIGHT_VELOCITY = 100; //millimeters per second
    //   public final double DISTANCE_VELOCITY = 100; //millimeters per second

    double length1;
    double length2;

    double[] retval2;
    double[] retval3;
    double[] testPoint;

    public final double MINIMUM_HEIGHT = -4*25.4;//-12 * 25.4; //millimeters
    public final double MAXIMUM_HEIGHT = 22* 25.4; //21 * 25.4; //millimeters
    //THESE TWO ARE DIFFERENT, REFER TO ARM1_LENGTH AND ARM2_LENGTH
    //public final double ARM1 = 18*25.4;   //millimeters
    // public final double ARM2 = 16*25.4; //millimeters

    /*
     * resetEncoders - flag indicating whether the encoders for the BaseArm and Intake arm should be reset
     * Autonomous will likely want to reset the encoders to 0 and then set the starting angle for both.
     * Teleop should not reset the encoders and instead calculate the current angle, height, and distance
     * using the current encoder values.
     */
    //DRIVE VARIABLES
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.5, 0, 0); // set to default 8 from 0 on quickstart quide
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7.5, 0, 0);  // set to default 8 from 0 on quickstart quide

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public SnappyHardware(HardwareMap hardwareMap, boolean resetEncocders) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        //INVERSE KINEMATICS STUFF
        ik = new InverseKinematicsSnap(ARM1_LENGTH, ARM2_LENGTH);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        //   imu = hardwareMap.get(BNO055IMU.class, "imu");
        //  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;             THIS WAS IMU BUT NOW ITS DISABLED
        //  imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");  // Hub 1, Port 0
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");  // Hub 1, port 1
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");  // Hub 1, port 2
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");  // Hub 1, port 3

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");  //Control Hub 2, Port 3
        DumpDoor = hardwareMap.get(Servo.class, "DumpDoor"); //port 1
        Pivot = hardwareMap.get(Servo.class, "Pivot");//port 2 pivot ADD THIS TO CONFIG OR ELSE EVEYTHING WILL EXPLODE!!!!

        RotationMotor = hardwareMap.get(DcMotorEx.class, "RotationMotor"); //Hub 2, Port 0
        BaseArm = hardwareMap.get(DcMotorEx.class, "BaseArm");    //Hub 2, Port 1
        IntakeArm = hardwareMap.get(DcMotorEx.class, "IntakeArm");  //Hub 2, Port 2

        CarsouselServo = hardwareMap.get(Servo.class, "carousel"); //port 0

        CarsouselServo.setDirection(Servo.Direction.FORWARD);

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        DumpDoor.setDirection(Servo.Direction.FORWARD);

        RotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BaseArm.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeArm.setDirection(DcMotorSimple.Direction.REVERSE);

        RotationMotor.setTargetPosition(0);
        RotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       RotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        if (resetEncocders) {
            BaseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BaseArm.setTargetPosition(0);
        } else {
            BaseArm.setTargetPosition(BaseArm.getCurrentPosition());
        }
        BaseArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (resetEncocders) {
            IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IntakeArm.setTargetPosition(0);
        } else {
            IntakeArm.setTargetPosition(IntakeArm.getCurrentPosition());
        }
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ik = new InverseKinematicsSnap(ARM1_LENGTH, ARM2_LENGTH);

        double currentBaseArmAngle = BaseArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
        double currentIntakeArmAngle = IntakeArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
        double[] coordinates = ik.getPoint(currentBaseArmAngle, currentIntakeArmAngle);
        distance = coordinates[0];
        height = coordinates[1];
        rotation = INITIAL_ROTATION_ANGLE;

        double lastHeight = height;

        double lastDistance = distance;

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection((DcMotorSimple.Direction.FORWARD));
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public void StopArmMovement() {

        RotationMotor.setPower(0.0);
        BaseArm.setPower(0.0);
        IntakeArm.setPower(0.0);
    }

    public void StopCarousel (){

        CarsouselServo.setPosition(0.5);
    }

    public void BlueSpin () {

        CarsouselServo.setPosition(1.0);
    }

    public void RedSpin () {

        CarsouselServo.setPosition(0.0);

    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }



    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    } //imu.getAngularOrientation().firstAngle REPLACED BY 0

    @Override
    public Double getExternalHeadingVelocity() {   //THIS DOUBLE ISNT BEING USED
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void wait(LinearOpMode opmode, double time) {
        long startTime = System.nanoTime();
        while ((opmode.opModeIsActive()) && startTime + time * 1e9 > System.nanoTime());
    }

    public void moveArmToPosition(LinearOpMode opmode, double rotation, double distance, double height, double wrist) {

        Pivot.setPosition(wrist);
        double[] angles = ik.getAngles(distance, height);
        RotationMotor.setTargetPosition((int) ((rotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));
        RotationMotor.setPower(RotationPower);
//     BaseArm.setTargetPosition((int) ((angles[0] - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1));
//     IntakeArm.setTargetPosition((int) ((angles[1] - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));

        BaseArm.setTargetPosition((int) ((angles[0] - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1));
        //arm.IntakeArm.setTargetPosition((int) ((angles[1] - arm.INITIAL_ARM2_ANGLE + (angles[0] - arm.INITIAL_ARM1_ANGLE) / arm.GEAR_RATIO_ARM2_STAGE) * arm.ENCODER_TICKS_PER_DEGREE_ARM2));
        IntakeArm.setTargetPosition((int) ((angles[1] - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));
        BaseArm.setPower(0.75);
        IntakeArm.setPower(0.75);

        while (opmode.opModeIsActive() &&
                (RotationMotor.isBusy() || BaseArm.isBusy() || IntakeArm.isBusy())) {

            //RotationMotor.setPower(RotationPower);
            RotationMotor.setPower(1.0);
            BaseArm.setPower(0.6);
            IntakeArm.setPower(0.6);
        }

    }

    public void deliverXblocks (LinearOpMode opMode, double rot, int position ) {


        if (position == 1) {
            moveArmToPosition(opMode, -23.77, 200, 370, 1);
            this.wait(opMode, 0.5);
            moveArmToPosition(opMode, rot, 530, 80, 1);
            this.wait(opMode, 1.0);
            moveArmToPosition(opMode, rot, 530, 80, 1);
            DumpDoor.setPosition(DumpPosition);
            IntakeMotor.setPower(-0.25);
            // snappy.followTrajectorySequence(trajectory2);
            this.wait(opMode, 2);
            IntakeMotor.setPower(0);
            DumpDoor.setPosition(ClosePosition);
            moveArmToPosition(opMode, -20, INITIAL_DISTANCE, INITIAL_HEIGHT, 1);
        }
        else if (position == 2) {
            moveArmToPosition(opMode, -23.77, 200, 370, .8);
            this.wait(opMode, 0.5);
            moveArmToPosition(opMode, rot, 200, 220, 0.9);
            this.wait(opMode, 1.0);
            moveArmToPosition(opMode, rot, 578, 210, 0.9);
            DumpDoor.setPosition(DumpPosition);
            IntakeMotor.setPower(-0.25);
            // snappy.followTrajectorySequence(trajectory2);
            this.wait(opMode, 2);
            IntakeMotor.setPower(0);
            DumpDoor.setPosition(ClosePosition);
            moveArmToPosition(opMode, -20, INITIAL_DISTANCE, INITIAL_HEIGHT, .8);
        }
        else {
            moveArmToPosition(opMode, -23.77, 200, 370, 0.4);
            this.wait(opMode, 0.5);
            moveArmToPosition(opMode, rot, 200, 370, 0.4);
            this.wait(opMode, 1.0);
            moveArmToPosition(opMode, rot, 670, 390, 0.4);
            DumpDoor.setPosition(DumpPosition);
            IntakeMotor.setPower(-0.25);
            // snappy.followTrajectorySequence(trajectory2);
            this.wait(opMode, 2);
            IntakeMotor.setPower(0);
            DumpDoor.setPosition(ClosePosition);
            moveArmToPosition(opMode, -20, INITIAL_DISTANCE, INITIAL_HEIGHT, 0.8);
        }

    }

}
