package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp(name = "GM TeleOp")
public class GMTeleop extends OpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor intake;
    DcMotor flywheel;
    DcMotor arm;
    Servo index;
    Servo botGrab;
    Servo topGrab;

    private BNO055IMU imu;

    private double liftPosScale = 10, liftPowScale = 0.005;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;

    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {

        // defining all the hardware
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        rightFront = hardwareMap.dcMotor.get("rf");
        intake = hardwareMap.dcMotor.get("intake");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        index = hardwareMap.servo.get("indexer");
        botGrab = hardwareMap.servo.get("bot_grab");
        topGrab = hardwareMap.servo.get("top_grab");
        arm = hardwareMap.dcMotor.get("arm");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // this puts the motors in reverse
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // enable encoders on motors that have them
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        float x1 = -gamepad1.right_stick_x;
        float y1 = gamepad1.left_stick_y;
        float r1 = gamepad1.right_trigger;
        float r2 = gamepad1.left_trigger;
        float f1 = gamepad2.right_stick_y;
        double power = 0;

        boolean indexRing = gamepad2.right_bumper;
        boolean takein = gamepad2.b;
        boolean takeout = gamepad2.a;
        boolean releaseWobble = gamepad2.left_bumper;

        telemetry.addData("Arm current position: ", arm.getCurrentPosition());
        telemetry.addData("Arm target position: ", liftPosDes);
        telemetry.update();

        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;

        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;
        rightFrontPower += y1;
        rightBackPower += y1;

        // Handle strafing movement
        leftFrontPower += x1;
        leftBackPower -= x1;
        rightFrontPower -= x1;
        rightBackPower += x1;

        // Handle clockwise turning movement
        leftFrontPower -= r1*0.50;
        leftBackPower -= r1*0.50;
        rightFrontPower += r1*0.50;
        rightBackPower += r1*0.50;

        // Handle counterclockwise turning movement
        leftFrontPower += r2*0.50;
        leftBackPower += r2*0.50;
        rightFrontPower -= r2*0.50;
        rightBackPower -= r2*0.50;

        if (takein == true) {
            intake.setPower(1.00);
        } else {
            intake.setPower(0.00);
        }
        if (takeout == true) {
            intake.setPower(-1.00);
        } else {
            intake.setPower(0.00);
        }
        if (indexRing == true) {
            index.setPosition(0.75);
        } else {
            index.setPosition(1.0);
        }
        if (releaseWobble == true) {
           botGrab.setPosition(0.40);
           topGrab.setPosition(0.35);
        } else {
            botGrab.setPosition(0.85);
            topGrab.setPosition(0.80);
        }
        if (f1 > 0) {
            power = 0.5;
            flywheel.setPower(power);
            if(power > 0) {
                double currentPower = flywheel.getPower();
                double error = (power - currentPower);
                flywheel.setPower(currentPower + error);
            }
        }
        if (f1 < 0) {
            flywheel.setPower(0.00);
        }

        liftPosCurrent = arm.getCurrentPosition();

        liftPosDes += liftPosScale*gamepad2.left_stick_y;                       //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
        //        integrater += liftPosError;                                   //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        arm.setPower(liftPow);

        // Scale movement
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.375, .375);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.375, .375);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.375, .375);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.375, .375);
        }

        leftRear.setPower(-leftBackPower);
        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);
        rightRear.setPower(-rightBackPower);
    }
}