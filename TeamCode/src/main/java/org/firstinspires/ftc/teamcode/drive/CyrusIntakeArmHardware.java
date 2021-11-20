package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CyrusIntakeArmHardware {

    public DcMotor IntakeMotor;
    public Servo DumpDoor;
    public Servo Pivot;
    public DcMotorEx RotationMotor;
    public DcMotorEx BaseArm;
    public DcMotorEx IntakeArm;
    public final double MINIMUM_ROTATION_ANGLE = -135.0; //degrees
    public final double MAXIMUM_ROTATION_ANGLE = 135.0; //degrees
    public final double INITIAL_ROTATION_ANGLE = -45.0; //degrees
    public final double INITIAL_ARM1_ANGLE = 216;//degree
    public final double INITIAL_ARM2_ANGLE = -180;//degree

    public double ENCODER_TICKS_PER_DEGREE_MOTOR = 28.0 / 360.0;
    public double GEARBOX_RATIO_ROTATION_MOTOR = 46.0 / 17.0 + 1;
    public double GEARBOX_RATIO_ARM1_MOTOR = (46.0 / 17.0 + 1) * (46.0 / 17.0 + 1);
    public double GEARBOX_RATIO_ARM2_MOTOR = (46.0 / 17.0 + 1);
    public double GEAR_RATIO_ROTATION_STAGE = (180.0 / 18.0);
    public double GEAR_RATIO_ARM1_STAGE = 20;
    public double GEAR_RATIO_ARM2_STAGE = 32;

    //ETPD CALCULATIONS
    public final double ENCODER_TICKS_PER_DEGREE_ROTATION = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ROTATION_MOTOR * GEAR_RATIO_ROTATION_STAGE;
    public final double ENCODER_TICKS_PER_DEGREE_ARM1 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM1_MOTOR *GEAR_RATIO_ARM1_STAGE;
    public final double ENCODER_TICKS_PER_DEGREE_ARM2 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM2_MOTOR * GEAR_RATIO_ARM2_STAGE;

    //ARM LENGTH
    public final double ARM1_LENGTH = 460.0; //millimeters
    public final double ARM2_LENGTH = 405.0; //millimeters

    public final double SAFE_POSITION_DISTANCE = -70;
    public final double SAFE_POSITION_HEIGHT = 20;

    //VELOCITIES
    //public final double ROTATION_VELOCITY = 50; //degrees per second
//    public final double HEIGHT_VELOCITY = 100; //millimeters per second
 //   public final double DISTANCE_VELOCITY = 100; //millimeters per second


    double length1;
    double length2;




    double [] retval2;
    double [] retval3;
    double [] testPoint;




    public final double MINIMUM_HEIGHT = -12*25.4; //millimeters
    public final double MAXIMUM_HEIGHT = 21*25.4; //millimeters
    //THESE TWO ARE DIFFERENT, REFER TO ARM1_LENGTH AND ARM2_LENGTH
    //public final double ARM1 = 18*25.4;   //millimeters
   // public final double ARM2 = 16*25.4; //millimeters


    /*
     * resetEncoders - flag indicating whether the encoders for the BaseArm and Intake arm should be reset
     * Autonomous will likely want to reset the encoders to 0 and then set the starting angle for both.
     * Teleop should not reset the encoders and instead calculate the current angle, height, and distance
     * using the current encoder values.
     */
    public CyrusIntakeArmHardware(HardwareMap hardwareMap, boolean resetEncocders){

        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");  //Control Hub 2, Port 3
        DumpDoor = hardwareMap.get(Servo.class, "DumpDoor"); //port 1
        Pivot = hardwareMap.get(Servo.class, "Pivot");//port 2 pivot ADD THIS TO CONFIG OR ELSE EVEYTHING WILL EXPLODE!!!!

        RotationMotor = hardwareMap.get(DcMotorEx.class, "RotationMotor"); //Hub 2, Port 0
        BaseArm = hardwareMap.get(DcMotorEx.class, "BaseArm");    //Hub 2, Port 1
        IntakeArm = hardwareMap.get(DcMotorEx.class, "IntakeArm");  //Hub 2, Port 2

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        DumpDoor.setDirection(Servo.Direction.FORWARD);

       RotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
       BaseArm.setDirection(DcMotorSimple.Direction.FORWARD);
       IntakeArm.setDirection(DcMotorSimple.Direction.REVERSE);



       RotationMotor.setTargetPosition(0);
       RotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       RotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       RotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (resetEncocders) {
            BaseArm.setTargetPosition(0);
            BaseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            BaseArm.setTargetPosition(BaseArm.getCurrentPosition());
        }
       BaseArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (resetEncocders) {
            IntakeArm.setTargetPosition(0);
            IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else {
            IntakeArm.setTargetPosition(IntakeArm.getCurrentPosition());
        }
       IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

        public void StopArmMovement () {

        IntakeMotor.setPower(0.0);
        DumpDoor.setPosition(0.0);
        RotationMotor.setPower(0.0);
        BaseArm.setPower(0.0);
        IntakeArm.setPower(0.0);

    }
        public void setEncoders () {
            IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BaseArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    public CyrusIntakeArmHardware(double arm1_length, double arm2_length) {
        length1 = arm1_length;
        length2 = arm2_length;

        retval2 = new double[2];
        retval3 = new double[3];
        testPoint = new double[2];
    }
    public double[] getAngles(double x, double y) {
        double tmp = (x*x + y*y - length1 * length1 - length2 * length2) / (2 * length1 * length2);
        double theta2 = Math.atan2(-Math.sqrt(1 - tmp * tmp), tmp);
        double k1 = length1 + length2 * Math.cos(theta2);
        double k2 = length2 * Math.sin(theta2);
        double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
        retval2[0] = Math.toDegrees(theta1);
        retval2[1] = Math.toDegrees(theta2);

        return retval2;
    }

    public double[] getPoint(double angle1, double angle2) {
        testPoint[0] = length1 * Math.cos(Math.toRadians(angle1)) + length2 * Math.cos(Math.toRadians(angle1 + angle2));
        testPoint[1] = length1 * Math.sin(Math.toRadians(angle1)) + length2 * Math.sin(Math.toRadians(angle1 + angle2));
        return testPoint;
    }

    public double[] getAngles(double x, double y, double z) {
        double [] angles = getAngles(Math.hypot(x, y), z);
        retval3[0] = Math.toDegrees(Math.atan2(y, x));
        retval3[1] = angles[0];
        retval3[2] = angles[1];

        return retval3;
    }
}
