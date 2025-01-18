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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp
public class zestyFieldCentricTeleop extends LinearOpMode {
    //    protected DcMotor rightSlide = null;
//    protected DcMotor leftSlide = null;
//    protected DcMotor sampleSlide = null;
//    protected Servo specimenClaw = null;
//    protected CRServo sampleIntake = null;
//    protected Servo sampleArm = null;
    private List<Action> runningActions = new ArrayList<>();
    Pose2d poseEstimate;
//    protected Servo bucket = null;
//    protected Servo samplePusher = null;



    @Override
    public void runOpMode() throws InterruptedException {



        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

//        rightSlide = hardwareMap.dcMotor.get("rightSlide");
//        leftSlide = hardwareMap.dcMotor.get("leftSlide");
//        //sampleSlide = hardwareMap.get(DcMotorEx.class,"sampleSlide");
//
//        //Bency : changed dcMotorEx to dcMotor
//
//        sampleSlide = hardwareMap.dcMotor.get("sampleSlide");
//
//        specimenClaw = hardwareMap.get(Servo.class,"specimenClaw");
//        sampleIntake = hardwareMap.get(CRServo.class, "sampleIntake");
//        sampleArm = hardwareMap.get(Servo.class,"sampleArm");
//        bucket = hardwareMap.get(Servo.class,"bucket");
//        samplePusher = hardwareMap.get(Servo.class,"samplePusher");

        waitForStart();
        while (opModeIsActive()){



//            if(gamepad2.dpad_right){
//                sampleIntake.setPower(1);
//                sampleArm.setPosition(0);
//                sleep(1000);
//                sampleIntake.setPower(-1);
//                samplePusher.setPosition(1);
//                sleep(100);
//                samplePusher.setPosition(-1);
//                sampleIntake.setPower(0);
//
//
//

            double heading = drive.pose.heading.toDouble();

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            double tempX = x * Math.cos(heading) - y * Math.sin(heading);
            double tempY = x * Math.sin(heading) + y * Math.cos(heading);

            if(gamepad1.left_bumper) {

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                tempX/2,
                                tempY/2
                        ),
                        rotation/2
                ));
                drive.updatePoseEstimate();



            }
            else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                tempX,
                                tempY
                        ),
                        rotation
                ));

                drive.updatePoseEstimate();
            }
            if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
                drive.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else{
                drive.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drive.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

            if (gamepad2.left_trigger > 0){
                if(gamepad2.circle){

                }
                else {
                    drive.sampleSlide.setPower(0);
                    drive.leftSlide.setPower(gamepad2.left_trigger);
                    drive.rightSlide.setPower(gamepad2.left_trigger);
                }

            }
            else{
                drive.leftSlide.setPower(0);
                drive.rightSlide.setPower(0);
                drive.sampleSlide.setPower(0);

            }
            if (gamepad2.right_trigger > 0){


                if(gamepad2.circle){
                      }
                else{
                    drive.sampleSlide.setPower(0);
                    drive.leftSlide.setPower(-gamepad2.right_trigger);
                    drive.rightSlide.setPower(-gamepad2.right_trigger);
                }
                if(drive.leftSlide.getCurrentPosition() < -2800){
                    drive.leftSlide.setPower(0);
                    drive.rightSlide.setPower(0);
                }
            }
            else {
                drive.leftSlide.setPower(0);
                drive.rightSlide.setPower(0);
            }

//            if(gamepad2.right_trigger > 0){
//
//                if(gamepad2.circle){
//                    drive.sampleSlide.setPower(gamepad2.right_trigger);
//                }
//                else{
//                    drive.sampleSlide.setPower(0);
//                    drive.leftSlide.setPower(-gamepad2.right_trigger);
//                    drive.rightSlide.setPower(-gamepad2.right_trigger);
//                }
//                if(drive.leftSlide.getCurrentPosition() < -2800){
//                    drive.leftSlide.setPower(0);
//                    drive.rightSlide.setPower(0);
//                }
//            }
//            else{
//                drive.leftSlide.setPower(0);
//                drive.rightSlide.setPower(0);
//                drive.sampleSlide.setPower(0);
//            }
//            if(gamepad2.left_trigger > 0){
//
//                if(gamepad2.circle){
//                    drive.sampleSlide.setPower(-gamepad2.left_trigger);
//
//                    drive.leftSlide.setPower(0);
//                    drive.rightSlide.setPower(0);
//                }
//                else{
//                    drive.sampleSlide.setPower(0);
//                    drive.leftSlide.setPower(gamepad2.left_trigger);
//                    drive.rightSlide.setPower(gamepad2.left_trigger);
//                }
//            }
//            else{
//                drive.leftSlide.setPower(0);
//                drive.rightSlide.setPower(0);
//                drive.sampleSlide.setPower(0);
//            }
            if(gamepad2.cross){
                drive.sampleIntake.setPower(1);
            }
            else {
                drive.sampleIntake.setPower(-1);
            }
            if (gamepad2.dpad_up){
                drive.sampleArm.setPosition(0);
            }
            if (gamepad2.dpad_down){
                drive.sampleArm.setPosition(1);
            }

            if (gamepad2.triangle){
                drive.bucket.setPosition(0);
            }
            if (gamepad2.square){
                drive.bucket.setPosition(1);
            }
            if (gamepad2.dpad_right){
                drive.samplePusher.setPosition(1);
            }
            if(gamepad2.dpad_left) {
                drive.samplePusher.setPosition(0);
            }
            if (gamepad2.left_bumper) {
                drive.specimenClaw.setPosition(1);
            }
            if (gamepad2.right_bumper) {
                drive.specimenClaw.setPosition(0);
            }





            telemetry.addData("sampleSlide power: ", drive.sampleSlide.getPower());
            telemetry.addData("rightSlide power: ", drive.rightSlide.getPower());
            telemetry.addData("leftSlide power: ", drive.leftSlide.getPower());
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
