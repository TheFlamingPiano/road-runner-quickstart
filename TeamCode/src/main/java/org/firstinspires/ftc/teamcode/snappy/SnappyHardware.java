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
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import com.qualcomm.robotcore.util.Range;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SnappyHardware extends MecanumDrive {



    //Carousel thingy
    public Servo CarsouselServo;

    //ARM HARDWARE
    public DcMotor CaraSlideMotor;
    public Servo IntakeServo;
    public Servo ClawServo;
    public Servo Pivot;
    public Servo FrontEncoderServo;
    public DcMotorEx RotationMotor;
    public DcMotorEx BaseArm;
    public DcMotorEx IntakeArm;
    public DistanceSensor sensorRange;

    public final double MINIMUM_ROTATION_ANGLE = -160.0; //degrees //-135
    public final double MAXIMUM_ROTATION_ANGLE = 160.0; //degrees
    public double INITIAL_ROTATION_ANGLE; //= -45.0 degrees
    public final double INITIAL_ARM1_ANGLE = 177.18; //180;//216;//degree
    public final double INITIAL_ARM2_ANGLE = -176.2; //-180;//degree

    public double ENCODER_TICKS_PER_DEGREE_MOTOR = 28.0 / 360.0;
    public double GEARBOX_RATIO_ROTATION_MOTOR = 1.0;
    public double GEARBOX_RATIO_ARM1_MOTOR = (46.0 / 11.0 + 1);
    public double GEARBOX_RATIO_ARM2_MOTOR = (46.0 / 11.0 + 1);
    public double GEAR_RATIO_ROTATION_STAGE = (140.0 / 16.0) * (80.0 / 20) * (60.0 / 15.0);
    public double GEAR_RATIO_ARM1_STAGE = 52;
    public double GEAR_RATIO_ARM2_STAGE = 32;
    //ang1+ang2secondbar + 90
    //ETPD CALCULATIONS
    public final double ENCODER_TICKS_PER_DEGREE_ROTATION = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ROTATION_MOTOR * GEAR_RATIO_ROTATION_STAGE;
    public final double ENCODER_TICKS_PER_DEGREE_ARM1 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM1_MOTOR * GEAR_RATIO_ARM1_STAGE;
    public final double ENCODER_TICKS_PER_DEGREE_ARM2 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM2_MOTOR * GEAR_RATIO_ARM2_STAGE;

    //ARM LENGTH
    public final double ARM1_LENGTH = 446.5; //460.0; //millimeters
    public final double ARM2_LENGTH = 468.2;//476.00;//405.0; //millimeters//438.2

    public final double INITIAL_HEIGHT = ARM1_LENGTH * Math.sin(Math.toRadians(INITIAL_ARM1_ANGLE)) + ARM2_LENGTH * Math.sin(Math.toRadians(INITIAL_ARM1_ANGLE + INITIAL_ARM2_ANGLE));
    public final double INITIAL_DISTANCE = ARM1_LENGTH * Math.cos(Math.toRadians(INITIAL_ARM1_ANGLE)) + ARM2_LENGTH * Math.cos(Math.toRadians(INITIAL_ARM1_ANGLE + INITIAL_ARM2_ANGLE));

    public final double SAFE_POSITION_DISTANCE = INITIAL_DISTANCE; //-70;
    public final double SAFE_POSITION_HEIGHT = INITIAL_HEIGHT; //20;
    public final double SAFE_ROTATION_ANGLE = -35; // need to test

    //THIS IS CODE FOR ARM THAT WAS ORIGINALLY IN TELE OP (HEIGHT, DISTANCE, ROTATION STUFF)
    double height;
    double distance;
    double rotation;

    double TargetRotationAngle;
    double TargetARM1Angle;
    double TargetARM2Angle;

    double RequestedWristAngle;

    double lastHeight;
    double lastDistance;

    double ARM_HEIGHT_VELOCITY = 450; //mm/s
    double ARM_DISTANCE_VELOCITY = 450; //mm/s
    double ARM_ROTATION_VELOCITY = 20; //Degrees

    InverseKinematicsSnap ik;

    double RotationPower = 1;

    double currentBaseArmAngle; //commented OUt because of line
    double currentIntakeArmAndle;

    double ARM1_POWER = 1;
    double ARM2_POWER = 1;

    double ClosePosition = 0.2;
    double IntakePosition = 0.45;
    double DumpPosition = 0.55;

    double wristAngleTarget;

    //VELOCITIES
    //public final double ROTATION_VELOCITY = 50; //degrees per second
//    public final double HEIGHT_VELOCITY = 100; //millimeters per second
    //   public final double DISTANCE_VELOCITY = 100; //millimeters per second

    double length1;
    double length2;

    double[] retval2;
    double[] retval3;
    double[] testPoint;

    public final double MINIMUM_HEIGHT = -4 * 25.4;//-12 * 25.4; //millimeters
    public final double MAXIMUM_HEIGHT = 22 * 25.4; //21 * 25.4; //millimeters
    //THESE TWO ARE DIFFERENT, REFER TO ARM1_LENGTH AND ARM2_LENGTH
    //public final double ARM1 = 18*25.4;   //millimeters
    // public final double ARM2 = 16*25.4; //millimeters

    public ArmData ArmMoveData;
    public boolean ArmMoveIsActive = false;


    public final double ENCODER_WHEEL_UP = 0.145;
    public final double ENCODER_WHEEL_DOWN = .68; //.685 was original


    /*
     * resetEncoders - flag indicating whether the encoders for the BaseArm and Intake arm should be reset
     * Autonomous will likely want to reset the encoders to 0 and then set the starting angle for both.
     * Teleop should not reset the encoders and instead calculate the current angle, height, and distance
     * using the current encoder values.
     */
    //DRIVE VARIABLES
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 0, 0); // set to default 8 from 0 on quickstart quide //7.5 before i changed to 8
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 0);  // set to default 8 from 0 on quickstart quide//7.5 before i changed to 8 to test it would fix moving away from wall

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

    public SnappyHardware(HardwareMap hardwareMap, boolean resetEncocders, TeamColor teamcolor) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        if (teamcolor == TeamColor.RED) {
            INITIAL_ROTATION_ANGLE = 145;
        } else {
            INITIAL_ROTATION_ANGLE = -145;
        }

        //INVERSE KINEMATICS STUFF
        ik = new InverseKinematicsSnap(ARM1_LENGTH, ARM2_LENGTH);

// decrease timeout from .5 sec to 0 sec to deliver more blocks
// only use for park not duck auto?
//        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
//                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
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

        CaraSlideMotor = hardwareMap.get(DcMotor.class, "CaraSlideMotor");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");  //Control Hub 2, Port 3
        ClawServo = hardwareMap.get(Servo.class, "ClawServo"); //port 1 control hub
        Pivot = hardwareMap.get(Servo.class, "Pivot");//port 2 pivot ADD THIS TO CONFIG OR ELSE EVEYTHING WILL EXPLODE!!!! control hub

        RotationMotor = hardwareMap.get(DcMotorEx.class, "RotationMotor"); //Hub 2, Port 0
        BaseArm = hardwareMap.get(DcMotorEx.class, "BaseArm");    //Hub 2, Port 1
        IntakeArm = hardwareMap.get(DcMotorEx.class, "IntakeArm");  //Hub 2, Port 2

        CarsouselServo = hardwareMap.get(Servo.class, "carousel"); //port 0

        FrontEncoderServo = hardwareMap.get(Servo.class, "FrontEncoderServo"); //port 3 control hub

        sensorRange = hardwareMap.get(DistanceSensor.class, "IntakeDistanceSensor");

        FrontEncoderServo.setDirection(Servo.Direction.FORWARD);

        Pivot.setDirection(Servo.Direction.REVERSE);
        CarsouselServo.setDirection(Servo.Direction.FORWARD);
        CaraSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        ClawServo.setDirection(Servo.Direction.FORWARD);

      //  RotationMotor.setTargetPosition(0);
        RotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BaseArm.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeArm.setDirection(DcMotorSimple.Direction.REVERSE);


//       RotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CaraSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // RotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (encoderPosition == EncoderPosition.DOWN) {
//            moveEncoderWheelDown();
//        }
//        else {
//            moveEncoderWheelUp();
//        }
        if (resetEncocders) {
            BaseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BaseArm.setTargetPosition(0);
        } else {
            BaseArm.setTargetPosition(BaseArm.getCurrentPosition());
            TargetARM1Angle = BaseArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
        }
        BaseArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (resetEncocders) {
            IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            IntakeArm.setTargetPosition(0);
        } else {
            IntakeArm.setTargetPosition(IntakeArm.getCurrentPosition());
            TargetARM2Angle = IntakeArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
        }
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (resetEncocders) {
            RotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RotationMotor.setTargetPosition(0);
        } else {
            RotationMotor.setTargetPosition(RotationMotor.getCurrentPosition());
            TargetRotationAngle = RotationMotor.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ROTATION + INITIAL_ROTATION_ANGLE;
        }
        RotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ik = new InverseKinematicsSnap(ARM1_LENGTH, ARM2_LENGTH);

        double currentBaseArmAngle = BaseArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
        double currentIntakeArmAngle = IntakeArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
        double[] coordinates = ik.getPoint(currentBaseArmAngle, currentIntakeArmAngle);
        distance = coordinates[0];
        height = coordinates[1];
        TargetRotationAngle = INITIAL_ROTATION_ANGLE;

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
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection((DcMotorSimple.Direction.REVERSE));
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

    public double getCurrentRotationAngle() {
        return RotationMotor.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ROTATION + INITIAL_ROTATION_ANGLE;
    }
    public double getCurrentBaseArmAngle() {
        return BaseArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
    }
    public double getCurrentIntakeArmAngle() {
        return IntakeArm.getCurrentPosition() * 1.0 / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
    }

    public void StopCarousel() {

        CarsouselServo.setPosition(0.5);
    }

    public void BlueSpin() {

        CarsouselServo.setPosition(1.0);
    }

    public void RedSpin() {

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

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
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
        while ((opmode.opModeIsActive()) && startTime + time * 1e9 > System.nanoTime()) ;
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

        long currentTime = System.nanoTime();

        while (opmode.opModeIsActive() && (System.nanoTime() < currentTime + 3e9) &&
                (RotationMotor.isBusy() || BaseArm.isBusy() || IntakeArm.isBusy())) {


            //RotationMotor.setPower(RotationPower);
            RotationMotor.setPower(1.0);
            BaseArm.setPower(0.6);
            IntakeArm.setPower(0.6);
        }

    }

//    public void deliverXblocks (LinearOpMode opMode, double rot, int position ) {
//
//                long numberOfSeconds = 1;
//
//
//        if (position == 1) {
//            StepBreakMovement(opMode, -23.77, 200, 370, 1, numberOfSeconds);
//            StepBreakMovement (opMode, rot, 530, 80, 1, numberOfSeconds);
//            StepBreakMovement(opMode, rot, 530, 80, 1, numberOfSeconds);
//            DumpDoor.setPosition(DumpPosition);
//            IntakeMotor.setPower(-0.25);
//            // snappy.followTrajectorySequence(trajectory2);
//            this.wait(opMode, 2);
//            IntakeMotor.setPower(0);
//            DumpDoor.setPosition(ClosePosition);
//            StepBreakMovement(opMode, -20, INITIAL_DISTANCE, INITIAL_HEIGHT, 1, numberOfSeconds);
//        }
//        else if (position == 2) {
//            StepBreakMovement(opMode, -23.77, 200, 370, .8, numberOfSeconds);
//            StepBreakMovement(opMode, rot, 200, 220, 0.9, numberOfSeconds);
//            StepBreakMovement(opMode, rot, 578, 210, 0.9, numberOfSeconds);
//            DumpDoor.setPosition(DumpPosition);
//            IntakeMotor.setPower(-0.25);
//            // snappy.followTrajectorySequence(trajectory2);
//            IntakeMotor.setPower(0);
//            DumpDoor.setPosition(ClosePosition);
//            StepBreakMovement(opMode, -20, INITIAL_DISTANCE, INITIAL_HEIGHT, 1, numberOfSeconds);
//        }
//        else {
//            StepBreakMovement(opMode, -23.77, 200, 370, 0.4, numberOfSeconds);
//            StepBreakMovement(opMode, rot, 200, 370, 0.4, numberOfSeconds);
//            StepBreakMovement(opMode, rot, 670, 390, 0.4, numberOfSeconds);
//            DumpDoor.setPosition(DumpPosition);
//            IntakeMotor.setPower(-0.25);
//            // snappy.followTrajectorySequence(trajectory2);
//            IntakeMotor.setPower(0);
//            DumpDoor.setPosition(ClosePosition);
//            StepBreakMovement(opMode, -20, INITIAL_DISTANCE, INITIAL_HEIGHT, 1, numberOfSeconds);
//        }
//
//    }

    public double setWristPosition(double wristAngle, double distance, double height) {
        // added return type for servo position
        double[] angles = ik.getAngles(distance, height);

        //   double wristAngleBar2 = ((wristAngle - (angles[0] - INITIAL_ARM1_ANGLE) - (angles[1] - INITIAL_ARM2_ANGLE)) / 180);
        double wristAngleBar2 = (-angles[0] - angles[1]) + wristAngle;

        //pivotposition 0.88 = 75 degrees
        //pivot position 0.15 = -70 degrees

        // TEST - replace this with built in calculation
//         double deltaAngle = 70+ 75;
//         double deltaPosition = 0.88-0.15;
//         double wristPosition = 0.15 + (deltaPosition/deltaAngle) * (wristAngleBar2 + 70);
        double AngleMin = -10; //-70;
        double AngleMax = 135; //75;
        double ServoMin = 0.166;
        double ServoMax = 0.9;
        double wristPosition = Range.scale(wristAngleBar2, AngleMin, AngleMax, ServoMin, ServoMax);

        RequestedWristAngle = wristAngle;

        Pivot.setPosition(wristPosition);
        return wristPosition;   //
    }

    public void StepBreakMovement(LinearOpMode opmode, double rotation, double distance, double height, double wristAng, double targetTime) {

        double currentTime = System.nanoTime() * 1.0;
        double startTime = currentTime;

        //setWristPosition(wristAng,height,distance);  // Cyrus.... you switched height and distance :-)
        setWristPosition(wristAng,distance,height);

        targetTime =  targetTime * 1e9;

        double[] angles = ik.getAngles(distance, height);

        int BaseArmCurrent = BaseArm.getCurrentPosition();
        int IntakeArmCurrent = IntakeArm.getCurrentPosition();
        int RotationArmCurrent = RotationMotor.getCurrentPosition();//rotation

        double currentBaseArmAngle = BaseArmCurrent / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
        double currentIntakeArmAngle = IntakeArmCurrent / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
        double currentRotationAngle = RotationArmCurrent / ENCODER_TICKS_PER_DEGREE_ROTATION + INITIAL_ROTATION_ANGLE;

        int BaseArmNew;
        int IntakeArmNew;
        int RotationArmNew;


        //double[] baseArmStart = ik.getPoint(currentBaseArmAngle,currentIntakeArmAngle);


//       int DeltaBaseArm = (int) ((BaseArmNew-BaseArmCurrent)/steps);
//       int DeltaIntakeArm = (int) ((IntakeArmNew-IntakeArmCurrent)/steps);

        BaseArm.setPower(1);
        IntakeArm.setPower(1);
        RotationMotor.setPower(1);
        while (currentTime < targetTime + startTime && opmode.opModeIsActive()) {

            currentTime = System.nanoTime();

            double elapsedTime = (currentTime - startTime);
            //double[] baseArmStart = ik.getPoint(BaseArmNew,IntakeArmNew);

            //double newHeight = baseArmStart[1] + (elapsedTime/targetTime)*(height - baseArmStart[1]);
            //double newDistance = current[0] + (elapsedTime/targetTime)*(distance - baseArmStart[0]);
            double newRotation = currentRotationAngle + (elapsedTime / targetTime) * (rotation - currentRotationAngle);
            double newBaseArmAngle = currentBaseArmAngle + (elapsedTime / targetTime) * (angles[0] - currentBaseArmAngle);
            double newIntakeArmAngle = currentIntakeArmAngle + (elapsedTime / targetTime) * (angles[1] - currentIntakeArmAngle);


            //this will be rotation, rotation angle will be same timeeslaped/target time *(rotation - rotationarmstart)

            // double[] newAngles = ik.getAngles(newDistance, newHeight);
            BaseArmNew = (int) ((newBaseArmAngle - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1);
            IntakeArmNew = ((int) ((newIntakeArmAngle - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));

            RotationArmNew = ((int) ((newRotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));


            RotationMotor.setTargetPosition(RotationArmNew);
            IntakeArm.setTargetPosition(IntakeArmNew);
            BaseArm.setTargetPosition(BaseArmNew);

            //while (opmode.opModeIsActive() && (System.nanoTime() < currentTime + 3e9) &&
            //  (RotationMotor.isBusy() || BaseArm.isBusy() || IntakeArm.isBusy())) {
            //RotationMotor.setPower(RotationPower);

        }

        BaseArmNew = (int) ((angles[0] - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1);
        IntakeArmNew = ((int) ((angles[1] - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));

        RotationArmNew = ((int) ((rotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));

        RotationMotor.setTargetPosition(RotationArmNew);
        IntakeArm.setTargetPosition(IntakeArmNew);
        BaseArm.setTargetPosition(BaseArmNew);

    }

    public void moveToPosition(LinearOpMode opmode, double rotation, double distance, double height, double wrist, double targetTime) {

        // all time is measured in seconds
        double currentTime = System.nanoTime() * 1.0e-9;
        double startTime = currentTime;

        setWristPosition(wrist,distance,height);
        double[] angles = ik.getAngles(distance, height);

        int BaseArmCurrent = BaseArm.getCurrentPosition();
        int IntakeArmCurrent = IntakeArm.getCurrentPosition();
        int RotationArmCurrent = RotationMotor.getCurrentPosition();//rotation

        int BaseArmNew = (int) ((angles[0] - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1);
        int IntakeArmNew = ((int) ((angles[1] - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));
        int RotationArmNew = ((int) ((rotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));

        // estimate the time to get from the current position to the new position
        double MOTOR_RPM = 4500;
        double TICKS_PER_SECOND = 28.0 * (MOTOR_RPM / 60);
        double MIN_MOTOR_POWER = 0.3;
        double baseArmTime = Math.abs(BaseArmNew - BaseArmCurrent) / TICKS_PER_SECOND;
        double intakeArmTime = Math.abs(IntakeArmNew - IntakeArmCurrent) / TICKS_PER_SECOND;
        double rotationTime = Math.abs(RotationArmNew - RotationArmCurrent) / TICKS_PER_SECOND;

        // scale power by the longest time
        double longestTime = baseArmTime;
        if (intakeArmTime > longestTime) longestTime = intakeArmTime;
        if (rotationTime > longestTime) longestTime = rotationTime;

        double baseArmPower = baseArmTime / longestTime;
        if (baseArmPower < MIN_MOTOR_POWER) baseArmPower = MIN_MOTOR_POWER;
        BaseArm.setTargetPosition(BaseArmNew);
        BaseArm.setPower(baseArmPower);

// always give the second arm full power
//        double intakeArmPower = intakeArmTime / longestTime;
        double intakeArmPower = 1.0;
        if (intakeArmPower < MIN_MOTOR_POWER) intakeArmPower = MIN_MOTOR_POWER;
        IntakeArm.setTargetPosition(IntakeArmNew);
        IntakeArm.setPower(intakeArmPower);

        double rotationPower = rotationTime / longestTime;
        if (rotationPower < MIN_MOTOR_POWER) rotationPower = MIN_MOTOR_POWER;
        RotationMotor.setTargetPosition(RotationArmNew);
        RotationMotor.setPower(rotationPower);

        // wait the estimated time
        while ((currentTime < targetTime + startTime) && (currentTime < longestTime *0.9 + startTime) && opmode.opModeIsActive()) {
            this.wait(opmode, 0.01);
            currentTime = System.nanoTime() * 1e-9;
        }

        // hold position
        BaseArm.setPower(1);
        IntakeArm.setPower(1);
        RotationMotor.setPower(1);
    }

    public void deliverXblocks(LinearOpMode opMode, double rot, int position, double distOffset) {

        double numberOfSeconds = .4;

        double waitTime = 2.0;

        if (position == 1) {
            double wristSpot;
            wristSpot = 0;

            double hubHeight;
            double heightOffset;
            if (distOffset > 100) {
                hubHeight = 40;
                heightOffset = 22;
            } else {
                hubHeight = 17;
                heightOffset = 0;
            }
            ClawServo.setPosition(0);
            //StepBreakMovement(opMode, -23.77, 91, -10, 0.4, numberOfSeconds);
            //  this.wait(opMode, 0.5);
            StepBreakMovement(opMode, rot, 200, 26+heightOffset, wristSpot, 1);
            //   this.wait(opMode, 0.5);
            StepBreakMovement(opMode, rot, 521 + distOffset, hubHeight, -20, numberOfSeconds);
             //this.wait(opMode, 1.0);
            IntakeServo.setPosition(0);
            ClawServo.setPosition(0.5);
            // snappy.followTrajectorySequence(trajectory2);
            waitForFreightNotDetected(opMode, .2);
//            this.wait(opMode, 1);
            IntakeServo.setPosition(0.5);
            ClawServo.setPosition(ClosePosition);
            StepBreakMovement(opMode, rot, 71, 30, 60, numberOfSeconds);
        } else if (position == 2) {
            double wristSpot;
            double hubHeight;

            wristSpot = 0;

            if (distOffset > 100) {
//                wristSpot = 0.21;
                hubHeight = 204;
            } else {
//                wristSpot = 0.28;
                hubHeight = 160;

            }
            ClawServo.setPosition(0);
            //StepBreakMovement(opMode, -23.77, 91, -10, 0.4, numberOfSeconds);
            //  this.wait(opMode, 0.5);
            //  this.wait(opMode, 0.5);
            StepBreakMovement(opMode, rot, 200, 209, wristSpot, 1);
            StepBreakMovement(opMode, rot, 565 + distOffset, hubHeight, -25, numberOfSeconds);
            //this.wait(opMode, 1.0);
            IntakeServo.setPosition(0);
            ClawServo.setPosition(0.5);
            // snappy.followTrajectorySequence(trajectory2);
            waitForFreightNotDetected(opMode, 1);
//            this.wait(opMode, 1);
//            StepBreakMovement(opMode, rot, 200, 110, 0.8, numberOfSeconds);
            IntakeServo.setPosition(0.5);
            ClawServo.setPosition(ClosePosition);
            StepBreakMovement(opMode, rot, 71, 30, 60, numberOfSeconds);
        } else {
            double hubHeight;
            if (distOffset > 100) {
                hubHeight = 383;
            } else {
                hubHeight = 388;
            }
            ClawServo.setPosition(0);
            //StepBreakMovement(opMode, rot, 91, -10, 1, numberOfSeconds);
            //  this.wait(opMode, 0.5);
//            moveToPosition(opMode, rot, 360 , 430, 1, numberOfSeconds + 0.50);
            StepBreakMovement(opMode, rot, 430 , 390, 50, numberOfSeconds + 0.40);
            //   this.wait(opMode, 0.5);
            //  this.wait(opMode, 1.0);
            StepBreakMovement(opMode, rot, 644 + distOffset/2, hubHeight, -23, numberOfSeconds + 0.4);
             // this.wait(opMode, 0.5);
            IntakeServo.setPosition(0);
            ClawServo.setPosition(0.5);
            waitForFreightNotDetected(opMode, .6);

//            double startTime = System.nanoTime() * 1e-9;
//            while ((opMode.opModeIsActive())
//                    && startTime + waitTime > System.nanoTime() * 1e-9
//                    && sensorRange.getDistance(DistanceUnit.MM) < 30.0);
//            // snappy.followTrajectorySequence(trajectory2);
//            this.wait(opMode, 0.85);
//            moveToPosition(opMode, rot, 500 , 370, 1, numberOfSeconds);
            IntakeServo.setPosition(0.5);
            ClawServo.setPosition(ClosePosition);
            StepBreakMovement(opMode, rot, 71 , 30, 60, numberOfSeconds);
        }

    }


    public void deliverExtraBlock(TeamColor teamColor, int i, LinearOpMode opMode, Pose2d startPos, Pose2d warehousePos) {
        double home_height = -22;
        double home_distance = 22;
        double deliverRotation, finalRotation;
        if (teamColor == TeamColor.BLUE){
            deliverRotation = -119;
            finalRotation = -10;
        } else {
            deliverRotation = 119;
            finalRotation = 10;
        }


        // drive into warehouse
        TrajectorySequence trajectory1 = trajectorySequenceBuilder(startPos)
                .back(27)
                .setVelConstraint(getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH))
                .back(10)
            .build();


        // lower the intake and turn on rollers
       // moveToPosition(opMode, 0-(i*6), 50, -56, -20, .8); // MAY NEED ADDED BACK IN IF ASYCH ARM MOVE DOESNT WORK :D
        NoneCodeBlockingArmMovement(opMode, 0-(i*6), 50, -56, -25, 0.9); // 0, 50, -60, -10
//        moveToPosition(opMode, 0, 60, -57, 1, 1);
        IntakeServo.setPosition(1);
        ClawServo.setPosition(0.3);
//        Pivot.setPosition(0.74);
//        Pivot.setPosition(0);

        // drive into the warehouse
        followTrajectorySequenceAsync(trajectory1);
//        double startTime = System.nanoTime() * 1e-9;
//        double timeout = 1.5;
//        boolean done = false;
//        double offset = 0;
//        double pivotOffset = 0.06;
//        while (opMode.opModeIsActive()
//                    && startTime + timeout > System.nanoTime() * 1e-9
//                    && !done) {
//            if (sensorRange.getDistance(DistanceUnit.MM) < 30.0) {
//                done = true;
//            } else {
//                offset += 25;
//                pivotOffset -=0.06;
//                moveToPosition(opMode, 0, 70 + offset, -60, -10, 1);
//                wait(opMode, 0.25);
//            }
//        }

        while (this.isBusy()) {
            UpdateArmMovement();
            this.update();
            if (!ArmMoveIsActive && sensorRange.getDistance(DistanceUnit.MM) < 30) {
               this.breakFollowing();
               break;
            }
        }


        // stop intake and close the claw
        IntakeServo.setPosition(0.5);
        ClawServo.setPosition(0);

        wait(opMode, 0.3);

        moveToPosition(opMode, finalRotation, home_distance , home_height, 40, .3);
        NoneCodeBlockingArmMovement(opMode, deliverRotation, home_distance , home_height, 70, 1.5);

        // drive out of warehouse
        TrajectorySequence trajectory2 = trajectorySequenceBuilder(getPoseEstimate())
                //.lineToLinearHeading(warehousePos)
                .lineToLinearHeading(startPos)
                .build();

        // drive out of the warehouse
        followTrajectorySequenceAsync(trajectory2);

        while (this.isBusy()){
            this.update();
            UpdateArmMovement();
        }

        // wait to clamp
//        wait(opMode,0.2);

        // try to rotate while driving
//        RotationMotor.setPower(0.5);
//        RotationMotor.setTargetPosition((int) ((deliverRotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));

        // drive out of the warehouse
        //followTrajectorySequence(trajectory2);


//       moveToPosition(opMode, deliverRotation, home_distance , home_height, 70, 0.5);
        moveToPosition(opMode, deliverRotation, 430 , 415, 70, 1);
        moveToPosition(opMode, deliverRotation, 644, 415, -14, 0.6);

        //wait(opMode, 0.4); // wait for arm to settle?
        // open claw and outtake rollers
        ClawServo.setPosition(0.5);
        IntakeServo.setPosition(0);
        waitForFreightNotDetected(opMode, 0.5);

        // is this needed?
//        this.wait(opMode, 0.85);

        IntakeServo.setPosition(0.5);
        ClawServo.setPosition(ClosePosition);
//        moveToPosition(opMode, deliverRotation, 71 , 30, 1, .4);
//        moveToPosition(opMode, finalRotation, 60, 0, 1, .5);
        moveToPosition(opMode, deliverRotation, 600, 395, 20, .2);
        moveToPosition(opMode, deliverRotation, 71 , 30, 40, .5);
        moveToPosition(opMode, finalRotation, home_distance , home_height, 50, .5);
    }


    public void waitForFreightDetected(LinearOpMode opmode, double timeout) {
        double startTime = System.nanoTime() * 1e-9;
//        while (opmode.opModeIsActive()
//                && startTime + 0.5 > System.nanoTime() * 1e-9);
        while (opmode.opModeIsActive()
                && startTime + timeout > System.nanoTime() * 1e-9
                && sensorRange.getDistance(DistanceUnit.MM) > 55.0);
    }

    public void waitForFreightNotDetected(LinearOpMode opmode, double timeout) {
        double startTime = System.nanoTime() * 1e-9;
        // wait a minimum of 0.5 seconds
        while (opmode.opModeIsActive()
                && startTime + 0.5 > System.nanoTime() * 1e-9);
        while (opmode.opModeIsActive()
                && startTime + timeout > System.nanoTime() * 1e-9
                && sensorRange.getDistance(DistanceUnit.MM) < 55.0);
    }

    public void setArmAnglesToHome(LinearOpMode opmode) {
        int RotationArmCurrent = RotationMotor.getCurrentPosition();//rotation
        double currentRotationAngle = RotationArmCurrent / ENCODER_TICKS_PER_DEGREE_ROTATION + INITIAL_ROTATION_ANGLE;

        StepBreakMovement(opmode, currentRotationAngle, 60 , 30, 70, .35);
    }


    public void StepBreakMovementX(LinearOpMode opmode, double rotation, double distance, double height, double wrist, long targetTime) {

        long currentTime = System.nanoTime();
        long startTime = currentTime;

        targetTime = (long) (targetTime * 1e9);

        Pivot.setPosition(wrist);
        double[] angles = ik.getAngles(distance, height);
        //RotationMotor.setTargetPosition((int) ((rotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));
        //RotationMotor.setPower(RotationPower);

        int BaseArmCurrent = BaseArm.getCurrentPosition();
        int IntakeArmCurrent = IntakeArm.getCurrentPosition();
        int RotationArmCurrent = RotationMotor.getCurrentPosition();//rotation

        double currentBaseArmAngle = BaseArmCurrent / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
        double currentIntakeArmAngle = IntakeArmCurrent / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
        double currentRotationAngle = RotationArmCurrent / ENCODER_TICKS_PER_DEGREE_ROTATION + INITIAL_ROTATION_ANGLE;

        int BaseArmNew;
        int IntakeArmNew;
        int RotationArmNew;


        double[] baseArmStart = ik.getPoint(currentBaseArmAngle, currentIntakeArmAngle);


//       int DeltaBaseArm = (int) ((BaseArmNew-BaseArmCurrent)/steps);
//       int DeltaIntakeArm = (int) ((IntakeArmNew-IntakeArmCurrent)/steps);

        BaseArm.setPower(0.75);
        IntakeArm.setPower(0.75);
        RotationMotor.setPower(1);
        while (currentTime < targetTime + startTime && opmode.opModeIsActive()) {

            currentTime = System.nanoTime();

            double elapsedTime = (currentTime - startTime);
            //double[] baseArmStart = ik.getPoint(BaseArmNew,IntakeArmNew);

            double newHeight = baseArmStart[1] + (elapsedTime / targetTime) * (height - baseArmStart[1]);
            double newDistance = baseArmStart[0] + (elapsedTime / targetTime) * (distance - baseArmStart[0]);
            double newRotation = currentRotationAngle + (elapsedTime / targetTime) * (rotation - currentRotationAngle);


            //this will be rotation, rotation angle will be same timeeslaped/target time *(rotation - rotationarmstart)

            double[] newAngles = ik.getAngles(newDistance, newHeight);
            BaseArmNew = (int) ((newAngles[0] - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1);
            IntakeArmNew = ((int) ((newAngles[1] - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));

            RotationArmNew = ((int) ((newRotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));


            RotationMotor.setTargetPosition(RotationArmNew);
            IntakeArm.setTargetPosition(IntakeArmNew);
            BaseArm.setTargetPosition(BaseArmNew);


            //while (opmode.opModeIsActive() && (System.nanoTime() < currentTime + 3e9) &&
            //  (RotationMotor.isBusy() || BaseArm.isBusy() || IntakeArm.isBusy())) {


            //RotationMotor.setPower(RotationPower);
            RotationMotor.setPower(1.0);
            BaseArm.setPower(0.6);
            IntakeArm.setPower(0.6);
        }
    }

    public void NoneCodeBlockingArmMovement(LinearOpMode opmode, double rotation, double distance, double height, double wrist, double targetTime) {

        ArmMoveData = new ArmData();

        ArmMoveData.opmode = opmode;

        ArmMoveData.currentTime = System.nanoTime();
        ArmMoveData.startTime = ArmMoveData.currentTime;

        ArmMoveData.targetTime = (long) (targetTime * 1e9);

       // Pivot.setPosition(wrist);
        setWristPosition(wrist,distance,height);
        double[] angles = ik.getAngles(distance, height);

        ArmMoveData.TargetIntakeArmAngle = angles[1];
        ArmMoveData.TargetBaseArmAngle = angles[0];

        ArmMoveData.rotation = rotation;

        ArmMoveData.BaseArmCurrent = BaseArm.getCurrentPosition();
        ArmMoveData.IntakeArmCurrent = IntakeArm.getCurrentPosition();
        ArmMoveData.RotationArmCurrent = RotationMotor.getCurrentPosition(); //rotation

        ArmMoveData.currentBaseArmAngle = ArmMoveData.BaseArmCurrent / ENCODER_TICKS_PER_DEGREE_ARM1 + INITIAL_ARM1_ANGLE;
        ArmMoveData.currentIntakeArmAngle = ArmMoveData.IntakeArmCurrent / ENCODER_TICKS_PER_DEGREE_ARM2 + INITIAL_ARM2_ANGLE;
        ArmMoveData.currentRotationAngle = ArmMoveData.RotationArmCurrent / ENCODER_TICKS_PER_DEGREE_ROTATION + INITIAL_ROTATION_ANGLE;

        BaseArm.setPower(1);
        IntakeArm.setPower(1);
        RotationMotor.setPower(1);

        ArmMoveIsActive = true;

    }

    public void UpdateArmMovement() {
        if (ArmMoveIsActive) {
            double currentTime = System.nanoTime();

            if (currentTime < ArmMoveData.targetTime + ArmMoveData.startTime && ArmMoveData.opmode.opModeIsActive()) {
                double elapsedTime = (currentTime - ArmMoveData.startTime);
                double newRotation = ArmMoveData.currentRotationAngle + (elapsedTime / ArmMoveData.targetTime) * (ArmMoveData.rotation - ArmMoveData.currentRotationAngle);
                double newBaseArmAngle = ArmMoveData.currentBaseArmAngle + (elapsedTime / ArmMoveData.targetTime) * (ArmMoveData.TargetBaseArmAngle - ArmMoveData.currentBaseArmAngle);
                double newIntakeArmAngle = ArmMoveData.currentIntakeArmAngle + (elapsedTime / ArmMoveData.targetTime) * (ArmMoveData.TargetIntakeArmAngle - ArmMoveData.currentIntakeArmAngle);

                //this will be rotation, rotation angle will be same timeeslaped/target time *(rotation - rotationarmstart)
                int BaseArmNew = (int) ((newBaseArmAngle - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1);
                int IntakeArmNew = ((int) ((newIntakeArmAngle - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));
                int RotationArmNew = ((int) ((newRotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));

                RotationMotor.setTargetPosition(RotationArmNew);
                IntakeArm.setTargetPosition(IntakeArmNew);
                ArmMoveData.opmode.telemetry.addData("new arm angle", newIntakeArmAngle);
                ArmMoveData.opmode.telemetry.addData("current arm angle", ArmMoveData.currentIntakeArmAngle);
                ArmMoveData.opmode.telemetry.addData("target arm angle", ArmMoveData.TargetIntakeArmAngle);
                ArmMoveData.opmode.telemetry.addData("read arm enc", IntakeArm.getCurrentPosition());

                BaseArm.setTargetPosition(BaseArmNew);

                RotationMotor.setPower(1.0);
                BaseArm.setPower(1.0);
                IntakeArm.setPower(1.0);
            } else {
                ArmMoveIsActive = false;
                int BaseArmNew = (int) ((ArmMoveData.TargetBaseArmAngle - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1);
                int IntakeArmNew = ((int) ((ArmMoveData.TargetIntakeArmAngle - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));
                int RotationArmNew = ((int) ((ArmMoveData.rotation - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));
                RotationMotor.setTargetPosition(RotationArmNew);
                IntakeArm.setTargetPosition(IntakeArmNew);
                BaseArm.setTargetPosition(BaseArmNew);

                TargetARM1Angle = ArmMoveData.TargetBaseArmAngle;
                TargetARM2Angle = ArmMoveData.TargetIntakeArmAngle;
                TargetRotationAngle = ArmMoveData.rotation;

            }
        } else {
            RotationMotor.setTargetPosition((int) ((TargetRotationAngle - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));
            BaseArm.setTargetPosition((int) ((TargetARM1Angle - INITIAL_ARM1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1));
            IntakeArm.setTargetPosition((int) ((TargetARM2Angle - INITIAL_ARM2_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM2));
        }
    }

    public void PrepickUpBlock(LinearOpMode opmode) {
        //StepBreakMovement(opmode, -10, 30, 10, 1, 0.3);
        StepBreakMovement(opmode, 0, 60, -57, -10, 1);
        IntakeServo.setPosition(1);
        ClawServo.setPosition(0.3);
//        Pivot.setPosition(-10);

    }

    public void postPickUpBlock (LinearOpMode opmode, TeamColor teamColor) {
        if (teamColor == TeamColor.BLUE) {
            StepBreakMovement(opmode, -10, 22, -22, 70, .4);
            StepBreakMovement(opmode, -119, 22, -22, 70, .4);
        } else {
            StepBreakMovement(opmode, 10, 22, -22, 70, .4);
        StepBreakMovement(opmode, 119, 22, -22, 70, .4);
    }
//        ClawServo.setPosition(0);
//        IntakeServo.setPosition(0.5);
        //StepBreakMovement(opmode, INITIAL_ROTATION_ANGLE,91,-10,1,1);

    }



 public void moveEncoderWheelUp() {
        FrontEncoderServo.setPosition(ENCODER_WHEEL_UP);
    }

    public void moveEncoderWheelDown() {
        FrontEncoderServo.setPosition(ENCODER_WHEEL_DOWN);
    }

    public enum TeamColor {
        RED,
        BLUE
    }

    public enum  EncoderPosition {
        UP,
        DOWN
    }

}
