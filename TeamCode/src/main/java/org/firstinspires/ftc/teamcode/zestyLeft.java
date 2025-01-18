package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous
public class zestyLeft extends LinearOpMode {

    private MecanumDrive mecanumDrive;
    public double tps = (180/60) * 537.6;

    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;
        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "rightSlide");
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2 = hardwareMap.get(DcMotorEx.class, "leftSlide");
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        public class LiftUp implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lift1.setVelocity(-tps);
                lift2.setVelocity(-tps);

                double pos = lift1.getCurrentPosition();
                if (pos > -3000){
                    return true;
                }
                else {

                    lift1.setVelocity(0);
                    lift2.setVelocity(0);
                    return false;
                }
            }
        }
        public Action LiftUp() {
            return new Lift.LiftUp();
        }


        public class LiftMed implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lift1.setVelocity(tps);
                lift2.setVelocity(tps);

                double pos = lift1.getCurrentPosition();
                if (pos < -800){
                    return true;
                }
                else {

                    lift1.setVelocity(0);
                    lift2.setVelocity(0);
                    return false;
                }
            }

        }
        public Action LiftMed() {
            return new Lift.LiftMed();
        }
        public class LiftDown implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lift1.setVelocity(tps);
                lift2.setVelocity(tps);
                initialized = true;

                double pos = lift1.getCurrentPosition();
                if (pos < -50){
                    return true;
                }
                else {

                    lift1.setVelocity(0);
                    lift2.setVelocity(0);
                    return false;
                }
            }

        }
        public Action LiftDown() {
            return new Lift.LiftDown();

        }
        public Action liftTo(int distance){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    int lift1Pos = lift1.getCurrentPosition();
                    int lift2Pos = lift2.getCurrentPosition();

                    lift1.setTargetPosition(distance);
                    lift2.setTargetPosition(distance);

                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    lift1.setVelocity(tps);
                    lift2.setVelocity(tps);
                    while (opModeIsActive() && lift1.isBusy() && lift2.isBusy()){
                        return true;
                    }
                    lift1.setVelocity(0);
                    lift2.setVelocity(0);
                    return false;
                }
            };
        }
    }
    public class sampleIntake {

        private CRServo sampleIntake;

        public sampleIntake(HardwareMap hardwareMap){
            sampleIntake = hardwareMap.get(CRServo.class,"sampleIntake");
        }
        public class sampleIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sampleIntake.setPower(1);
                return false;
            }
        }
        public Action sampleIn() {
            return new sampleIn();
        }

        public class sampleOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sampleIntake.setPower(-1);
                return false;
            }
        }
        public Action sampleOut() {
            return new sampleOut();
        }
    }

    public class bucket {
        private Servo bucket;
        public bucket(HardwareMap hardwareMap){
            bucket = hardwareMap.get(Servo.class,"bucket");
        }

        public class bucketOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(0);
                return false;
            }
        }

        public Action bucketOut() {
            return new bucketOut();
        }

        public class bucketIn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(1);
                return false;
            }
        }
        public Action bucketIn() {
            return new bucketIn();
        }
    }

    public class sampleArm {
        private Servo sampleArm;
        public sampleArm(HardwareMap hardwareMap){
            sampleArm = hardwareMap.get(Servo.class,"sampleArm");
        }
        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sampleArm.setPosition(0);
                return false;
            }
        }
        public Action ArmUp() {
            return new ArmUp();
        }
        public class ArmDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sampleArm.setPosition(1);
                return false;
            }
        }
        public Action ArmDown() {
            return new ArmDown();
        }

    }

    public class samplePusher {
        private Servo samplePusher;
        public samplePusher(HardwareMap hardwareMap){
            samplePusher = hardwareMap.get(Servo.class,"samplePusher");
        }

        public class PushOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                samplePusher.setPosition(0);
                return false;
            }
        }
        public Action PushOut(){
            return new PushOut();
        }

        public class PushIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                samplePusher.setPosition(1);
                return false;
            }

        }
        public Action PushIn(){

            return new PushIn();
        }

    }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-15,-63,Math.toRadians(00));
        MecanumDrive drive = new MecanumDrive(this.hardwareMap,initialPose);
        //specimenGripper gripper = new specimenGripper(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        sampleArm sampleArm = new sampleArm(hardwareMap);
        sampleIntake sampleIntake = new sampleIntake(hardwareMap);
        samplePusher samplePusher = new samplePusher(hardwareMap);
        bucket bucket = new bucket(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket(true);

        TrajectoryActionBuilder act1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-56,-56, Math.toRadians(45)), Math.toRadians(225))
                ;

//        TrajectoryActionBuilder act2 = drive.actionBuilder(new Pose2d(-56,-56,Math.toRadians(225)))
//                .splineToConstantHeading(new Vector2d(-58,-58),Math.toRadians(225))
//                ;

        TrajectoryActionBuilder act3 = drive.actionBuilder(new Pose2d(-58,-58, Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-35,-24, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-42,-16,Math.toRadians(70)), Math.toRadians(160))
                .setTangent(Math.toRadians(250))
                .lineToY(-56)
                .setTangent(Math.toRadians(55))
                .splineToLinearHeading(new Pose2d(-52,-15,Math.toRadians(80)), Math.toRadians(170))
                .setTangent(Math.toRadians(260))
                .lineToY(-53)
                .setTangent(Math.toRadians(75))
                .splineToLinearHeading(new Pose2d(-32,-11, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(0)
                .lineToX(-24)
                ;

        Action armUp = sampleArm.ArmUp();
        Action armDown = sampleArm.ArmDown();
        Action sampleIn = sampleIntake.sampleIn();
        Action sampleOut = sampleIntake.sampleOut();
        Action pushIn = samplePusher.PushIn();
        Action pushOut = samplePusher.PushOut();
        Action bucketOut = bucket.bucketOut();
        Action bucketIn = bucket.bucketIn();
        Action liftUp = lift.LiftUp();
        Action liftMed = lift.LiftMed();
        Action liftDown = lift.LiftDown();

        Actions.runBlocking(bucketOut);
        waitForStart();

        Action action1 = act1.build();
//        Action action2 = act2.build();
        Action action3 = act3.build();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                action1,
                                liftUp
                                ),
                        bucketIn,
                        new SleepAction(1.5),

                        new ParallelAction(
                                action3,
                                new SequentialAction(
                                new SleepAction(1),
                                        liftDown
                                )

                                )
                        )
                );





    }


}
