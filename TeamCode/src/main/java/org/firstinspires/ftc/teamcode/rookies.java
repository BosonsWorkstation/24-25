package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "rookies", group = "Linear Opmode")
public class rookies extends LinearOpMode{

    protected DcMotor front_left_wheel = null;
    protected DcMotor front_right_wheel = null;
    protected DcMotor back_left_wheel = null;
    protected DcMotor back_right_wheel = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine(String.format("alkdfj adj flakj ;lak ;lafd"));

        front_left_wheel = hardwareMap.dcMotor.get("front_left_wheel");
        back_left_wheel = hardwareMap.dcMotor.get("back_left_wheel");
        front_right_wheel = hardwareMap.dcMotor.get("front_right_wheel");
        back_right_wheel = hardwareMap.dcMotor.get("back_right_wheel");


        int stop = 0;
        double turnSpeed = 0.8;
        boolean uping = true;
        double power = 0.9;
        double crabPower = 1;

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.right_stick_x > .4) {
                front_left_wheel.setPower(-turnSpeed);
                back_left_wheel.setPower(-turnSpeed);
                front_right_wheel.setPower(-turnSpeed);
                back_right_wheel.setPower(turnSpeed);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.right_stick_x < -.4) {
                front_left_wheel.setPower(turnSpeed);
                back_left_wheel.setPower(turnSpeed);
                front_right_wheel.setPower(turnSpeed);
                back_right_wheel.setPower(-turnSpeed);

            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.left_stick_y < -.4) {
                front_left_wheel.setPower(-power);
                back_left_wheel.setPower(-power);
                front_right_wheel.setPower(power);
                back_right_wheel.setPower(-power);
                telemetry.addData("Front right: ", front_right_wheel.getPower());
                telemetry.addData("Front left: ", front_left_wheel.getPower());
                telemetry.addData("back right: ", back_right_wheel.getPower());
                telemetry.addData("back left: ", back_left_wheel.getPower());
                telemetry.update();
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);

            }

            if (gamepad1.left_stick_y > .4) {
                front_left_wheel.setPower(power);
                back_left_wheel.setPower(power);
                front_right_wheel.setPower(-power);
                back_right_wheel.setPower(power);
                telemetry.addData("Front right: ", front_right_wheel.getPower());
                telemetry.addData("Front left: ", front_left_wheel.getPower());
                telemetry.addData("back right: ", back_right_wheel.getPower());
                telemetry.addData("back left: ", back_left_wheel.getPower());
                telemetry.update();
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.left_stick_x > 0.4) {
                front_left_wheel.setPower(-crabPower);
                back_left_wheel.setPower(crabPower);
                front_right_wheel.setPower(-crabPower);
                back_right_wheel.setPower(-crabPower);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);

            }
            if (gamepad1.left_stick_x < -0.4) {
                front_left_wheel.setPower(crabPower);
                back_left_wheel.setPower(-crabPower);
                front_right_wheel.setPower(crabPower);
                back_right_wheel.setPower(crabPower);
            } else {
                front_left_wheel.setPower(stop);
                back_left_wheel.setPower(stop);
                front_right_wheel.setPower(stop);
                back_right_wheel.setPower(stop);
            }
            if (gamepad1.left_bumper) {
                power = 0.4;
                turnSpeed = 0.4;
                crabPower = 0.6;
            } else {
                power = 1.0;
                turnSpeed = 0.8;
                crabPower = 1.0;
            }


        }
    }











}
