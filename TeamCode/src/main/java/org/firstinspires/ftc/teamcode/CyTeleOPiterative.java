package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOP", group = "OPmodes")
public class CyTeleOPiterative extends OpMode {

    CyHardware robot = new CyHardware();

    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry.addData("HI", "IM READY");    //
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {
        double FowardBack;
        double Strafing;
        double Rotating;

        FowardBack = -gamepad1.left_stick_y;
        Strafing = gamepad1.left_stick_x;
        Rotating = gamepad1.right_stick_x;

        robot.TopLeftMotor.setPower(FowardBack + Strafing + Rotating  );
        robot.TopRightMotor.setPower(FowardBack - Strafing - Rotating);
        robot.BottomLeftMotor.setPower(FowardBack - Strafing + Rotating);
        robot.BottomRightMotor.setPower(FowardBack + Strafing - Rotating);
    }
        @Override
        public void stop (){
            robot.stopallmotors();
        }

}