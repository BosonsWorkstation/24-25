package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class newTele extends LinearOpMode {

    private List<Action> runningActions = new ArrayList<>();





    @Override
    public void runOpMode() throws InterruptedException {

        boolean sampleIntakeToggle = false;
        boolean bucketToggle = false;
        boolean samplePusherToggle = false;
        boolean specimenClawToggle = false;


        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));



        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        waitForStart();
        while (opModeIsActive()){




            if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
                drive.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{
                drive.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drive.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }


            if (gamepad2.left_trigger > 0){

                drive.leftSlide.setPower(gamepad2.left_trigger);
                drive.rightSlide.setPower(gamepad2.left_trigger);

            }
            else{
                drive.leftSlide.setPower(0);
                drive.rightSlide.setPower(0);

            }

            if (gamepad2.right_trigger > 0){

                drive.leftSlide.setPower(-gamepad2.right_trigger);
                drive.rightSlide.setPower(-gamepad2.right_trigger);

                if(drive.leftSlide.getCurrentPosition() < -2900){
                    drive.leftSlide.setPower(0);
                    drive.rightSlide.setPower(0);
                }
            }
            else {
                drive.leftSlide.setPower(0);
                drive.rightSlide.setPower(0);
            }


            if (gamepad2.right_bumper){
                drive.sampleSlide.setPower(-1);
            }
            else{
                drive.sampleSlide.setPower(0);
            }

            if (gamepad2.left_bumper){
                drive.sampleSlide.setPower(1);
            }
            else{
                drive.sampleSlide.setPower(0);
            }


            if(getRuntime() == 5){
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }


            if (gamepad2.dpad_up){
                drive.sampleArm.setPosition(0);
            }

            if(gamepad2.dpad_left){
                drive.sampleArm.setPosition(0.75);
            }

            if (gamepad2.dpad_down){
                drive.sampleArm.setPosition(1);
            }


            if(gamepad2.cross){

                drive.sampleIntake.setPower(1);
            }
            else{

                drive.sampleIntake.setPower(-1);

            }


            if (currentGamepad2.circle && !previousGamepad2.circle){
                bucketToggle = !bucketToggle;
            }

            if(bucketToggle){
                drive.bucket.setPosition(1);
            }
            else{
                drive.bucket.setPosition(0);
            }


            if (currentGamepad2.square && !previousGamepad2.square){
                samplePusherToggle = !samplePusherToggle;
            }

            if(samplePusherToggle){
                drive.samplePusher.setPosition(0);
            }
            else{
                drive.samplePusher.setPosition(1);
            }


            if(currentGamepad2.triangle && !previousGamepad2.triangle){
                specimenClawToggle = !specimenClawToggle;
            }

            if (specimenClawToggle) {
                drive.specimenClaw.setPosition(1);
            }
            else{
                drive.specimenClaw.setPosition(0);
            }

            if(gamepad1.cross){
                drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }




            telemetry.addData("triangle: ","bucket");
            telemetry.addData("square: ","sample pusher");
            telemetry.addData("cross: ","sample intake");
            telemetry.addData("circle: ","specimen claw");
            telemetry.addData("sampleSlide power: ", drive.sampleSlide.getPower());
            telemetry.addData("rightSlide power: ", drive.rightSlide.getPower());
            telemetry.addData("leftSlide pos: ", drive.leftSlide.getCurrentPosition());
            telemetry.addData("rightSlide pos: ", drive.rightSlide.getCurrentPosition());
            telemetry.addData("leftSlide power: ", drive.leftSlide.getPower());
            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;



        }


    }


}
