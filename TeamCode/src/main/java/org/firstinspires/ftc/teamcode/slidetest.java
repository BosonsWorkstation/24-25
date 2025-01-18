package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@TeleOp
public class slidetest extends LinearOpMode{

    protected DcMotorEx rightSlide = null;
    protected DcMotorEx leftSlide = null;
    protected DcMotorEx sampleSlide = null;

    public void runOpMode() throws InterruptedException{

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        rightSlide = hardwareMap.get(DcMotorEx.class,"rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class,"leftSlide");
        sampleSlide = hardwareMap.get(DcMotorEx.class,"sampleSlide");
        sampleSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{ leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

            if (gamepad1.left_trigger > 0){
                if(gamepad1.circle){
                    sampleSlide.setPower(gamepad1.left_trigger);

                }
                else {
                    sampleSlide.setPower(0);
                    leftSlide.setPower(gamepad1.left_trigger);
                    rightSlide.setPower(gamepad1.left_trigger);
                }

            }
            else{
                leftSlide.setPower(0);
                rightSlide.setPower(0);

            }
            if (gamepad1.right_trigger > 0){


                if(gamepad1.circle){
                    sampleSlide.setPower(-gamepad1.right_trigger);
                }
                else{
                    sampleSlide.setPower(0);
                    leftSlide.setPower(-gamepad1.right_trigger);
                    rightSlide.setPower(-gamepad1.right_trigger);
                }
                if(drive.rightSlide.getCurrentPosition() < -2800){
                    drive.leftSlide.setPower(0);
                    drive.rightSlide.setPower(0);
                }
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
            if(gamepad1.left_bumper){
                sampleSlide.setPower(1);
            }
            else{
                sampleSlide.setPower(0);
            }
            if(gamepad1.right_bumper){
                sampleSlide.setPower(-1);
            }else{
                sampleSlide.setPower(0);
            }


            telemetry.addData("left power: ", leftSlide.getPower());
            telemetry.addData("right power: ",rightSlide.getPower());
            telemetry.addData("sample power: ", sampleSlide.getPower());

            telemetry.addData("left velo: ", leftSlide.getVelocity());
            telemetry.addData("right velo: ",rightSlide.getVelocity());
            telemetry.addData("sample velo: ", sampleSlide.getVelocity());
            telemetry.addData("left pos: ", leftSlide.getCurrentPosition());
            telemetry.addData("right pos: ", rightSlide.getCurrentPosition());
            telemetry.addData("sample pos: ",sampleSlide.getCurrentPosition());

            telemetry.update();
        }
    }
}