package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ArmCode")
public class ArmCode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotor arm;
        arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double liftPosScale = 10, liftPowScale = 0.005;
        double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;

        waitForStart();

        while (opModeIsActive())
        {
            arm.setPower(gamepad2.left_stick_y);

            telemetry.addData("armPos", arm.getCurrentPosition());
            telemetry.update();
        }
//        liftPosCurrent = arm.getCurrentPosition();
//
//        liftPosDes += liftPosScale*gamepad2.left_stick_y;                //input scale factor
//        liftPosError = liftPosDes - liftPosCurrent;
////        integrater += liftPosError;                                           //unnecessary
//        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
//        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
//        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
//        arm.setPower(liftPow);
    }
}
