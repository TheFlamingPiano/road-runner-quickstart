package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Inverse Kinematics", group="Test")
public class TestInverseKinematics extends OpMode {
    final double ENCODER_TICKS_PER_DEGREE_MOTOR = 28.0 / 360;
    final double GEARBOX_RATIO_ROTATION_MOTOR = 46.0 / 17 + 1;
    final double GEARBOX_RATIO_ARM1_MOTOR = (46.0 / 17 + 1) * (46.0 / 17 + 1);
    final double GEARBOX_RATIO_ARM2_MOTOR = (46.0 / 17 + 1);
    final double GEAR_RATIO_ROTATION_STAGE = 180.0 / 18;
    final double GEAR_RATIO_ARM1_PIVOT = 20.0;
    final double GEAR_RATIO_ARM2_PIVOT = 32.0;
    final double ENCODER_TICKS_PER_DEGREE_ROTATION = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ROTATION_MOTOR * GEAR_RATIO_ROTATION_STAGE;
    final double ENCODER_TICKS_PER_DEGREE_ARM1 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM1_MOTOR * GEAR_RATIO_ARM1_PIVOT;
    final double ENCODER_TICKS_PER_DEGREE_ARM2 = ENCODER_TICKS_PER_DEGREE_MOTOR * GEARBOX_RATIO_ARM2_MOTOR * GEAR_RATIO_ARM2_PIVOT;

    final double ROTATION_VELOCITY = 90.0; // degrees per second
    final double EXTENSION_VELOCITY = 100.0; // mm per second
    final double HEIGHT_VELOCITY = 100.0; // mm per second

    final double LENGTH_ARM1 = 460; // mm
    final double LENGTH_ARM2 = 405; // mm

    final double INITIAL_ROTATION_ANGLE = 45.0; //degrees
    final double INITIAL_PIVOT1_ANGLE = 180.0; //degrees
    final double INITIAL_PIVOT2_ANGLE = -180.0; //degrees

    double currentTime;
    double lastTime;
    InverseKinematics ik;

    double rotationAngle = 0.0;
    double height = 0.0;
    double extension = 0.0;

    DcMotor rotationMotor;
    DcMotor pivot1Motor;
    DcMotor pivot2Motor;

    @Override
    public void init() {
        ik = new InverseKinematics(LENGTH_ARM1, LENGTH_ARM2);

        rotationMotor = hardwareMap.dcMotor.get("Rotation");
        rotationMotor.setDirection(DcMotor.Direction.REVERSE);
        rotationMotor.setTargetPosition(0);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(0.0);

        pivot1Motor = hardwareMap.dcMotor.get("Pivot 1");
        pivot1Motor.setDirection(DcMotor.Direction.FORWARD);
        pivot1Motor.setTargetPosition(0);
        pivot1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot1Motor.setPower(0.0);

        pivot2Motor = hardwareMap.dcMotor.get("Pivot 2");
        pivot2Motor.setDirection(DcMotor.Direction.REVERSE);
        pivot2Motor.setTargetPosition(0);
        pivot2Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot2Motor.setPower(0.0);

        // use forward kinematics to set initial height and extension
        rotationAngle = INITIAL_ROTATION_ANGLE;
        double[] point = ik.getPoint(INITIAL_PIVOT1_ANGLE, INITIAL_PIVOT2_ANGLE);
        extension = point[0];
        height = point[1];
    }

    @Override
    public void start() {
        super.start();
        currentTime = getRuntime();

//        rotationMotor.setPower(1.0);
        pivot1Motor.setPower(1.0);
        pivot2Motor.setPower(1.0);
    }

    @Override
    public void loop() {
        currentTime = getRuntime();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        rotationAngle += gamepad1.left_stick_x * ROTATION_VELOCITY * deltaTime;
        extension += -gamepad1.left_stick_y * EXTENSION_VELOCITY * deltaTime;
        height += -gamepad1.right_stick_y * HEIGHT_VELOCITY * deltaTime;

        double[] angles = ik.getAngles(extension, height);
        //double[] point = ik.getPoint(angles[0], angles[1]);  // should be the same as (extension, height) if everything is working correctly

        rotationMotor.setTargetPosition((int)((rotationAngle - INITIAL_ROTATION_ANGLE) * ENCODER_TICKS_PER_DEGREE_ROTATION));
        pivot1Motor.setTargetPosition((int) ((angles[0] - INITIAL_PIVOT1_ANGLE) * ENCODER_TICKS_PER_DEGREE_ARM1));
        // the target angle needs to be adjusted by the rotation of the first pivot since just moving the first arm
        // will change the angle of the second pivot.  The sign of the adjustment depends on which way the hinge was attached
        pivot2Motor.setTargetPosition((int) ((angles[1] - INITIAL_PIVOT2_ANGLE + (angles[0] - INITIAL_PIVOT1_ANGLE) / GEAR_RATIO_ARM2_PIVOT) * ENCODER_TICKS_PER_DEGREE_ARM2));
    }
}
