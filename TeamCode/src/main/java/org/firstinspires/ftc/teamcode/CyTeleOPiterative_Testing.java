package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOP-TEST", group = "OPmodes")
public class CyTeleOPiterative_Testing extends OpMode {

    CyHardware robot = new CyHardware();
    DataLogger dlog = new DataLogger("DriveLog", true);

    @Override
    public void init() {

        robot.init(hardwareMap);
        //dlog.StartLogging();
        //dlog.Log(new String[] {"JForwardBack", "JStrafe", "JRotate", " TL", "TR" , "BL", "BR", "TLnorm", "TRnorm", "BLnorm", "BRnorm"});

        telemetry.addData("HI", "IM READY");    //
    }

    @Override
    public void loop() {
        double FowardBack;
        double Strafing;
        double Rotating;

        FowardBack = -gamepad1.left_stick_y;
        Strafing = gamepad1.left_stick_x;
        Rotating = gamepad1.right_stick_x;

        double TL = FowardBack + Strafing + Rotating;
        double TR = FowardBack - Strafing - Rotating;
        double BL = FowardBack - Strafing + Rotating;
        double BR = FowardBack + Strafing - Rotating;

        // Normalize the input so Max is 1
        double maxMotor = Math.max(Math.abs(TL), Math.max(Math.abs(TR), Math.max(Math.abs(BL), Math.max(Math.abs(BR), 1.0))));
        double TLnorm = TL/maxMotor;
        double TRnorm = TR/maxMotor;
        double BLnorm = BL/maxMotor;
        double BRnorm = BR/maxMotor;

        robot.TopLeftMotor.setPower(TLnorm);
        robot.TopRightMotor.setPower(TRnorm);
        robot.BottomLeftMotor.setPower(BLnorm);
        robot.BottomRightMotor.setPower(BRnorm);

        //dlog.Log(new Double[] {FowardBack, Strafing, Rotating, TL, TR, BL, BR, TLnorm, TRnorm, BLnorm, BRnorm});
    }
        @Override
        public void stop (){
            robot.stopallmotors();
            //dlog.StopLogging();
        }

}