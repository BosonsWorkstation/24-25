package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@TeleOp
@Disabled
public class test1 extends LinearOpMode{

    protected DcMotorEx testSlide = null;

    public void runOpMode() throws InterruptedException{

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        testSlide = hardwareMap.get(DcMotorEx.class,"testSlide");


        testSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.dpad_up){

                testSlide.setPower(-1);


            }
            else{

testSlide.setPower(0);
            }
            if(gamepad1.dpad_down){

                testSlide.setPower(1);

            }
            else{
                testSlide.setPower(0);



            }

        }

    }



}
