package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous
public class simpleRight extends LinearOpMode {

    private MecanumDrive mecanumDrive;
    public double tps = 510 * (312/60);
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
                    if (pos > -1350){
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
            return new LiftUp();
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
            return new LiftMed();
        }
        public class LiftDown implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                    lift1.setVelocity(tps);
                    lift2.setVelocity(tps);

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
            return new LiftDown();

        }

    }

    public class specimenGripper {
        private Servo specimenGripper;
        public specimenGripper(HardwareMap hardwareMap){
            specimenGripper = hardwareMap.get(Servo.class,"specimenClaw");
        }

        public class CloseGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specimenGripper.setPosition(0);
                return false;
            }
        }
        public Action closeGripper(){
            return new CloseGripper();
        }

        public class OpenGripper implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                specimenGripper.setPosition(1);
                return false;
            }
        }
        public Action openGripper(){
            return new OpenGripper();
        }
    }











    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(15,-63,Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(this.hardwareMap,initialPose);
        //specimenGripper gripper = new specimenGripper(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        specimenGripper gripper = new specimenGripper(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket(true);
        drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectoryActionBuilder act1 = drive.actionBuilder(initialPose)
                .setTangent(90)
                //drop off preset specimen
                .splineToConstantHeading(new Vector2d(-3,-32),Math.toRadians(90), new TranslationalVelConstraint(70))
                //bring down slide then open claw
                ;

        TrajectoryActionBuilder act2 = drive.actionBuilder(new Pose2d(-3,-32,Math.toRadians(270)))
//                //move to pos while lowering slide
//                .setTangent(java.lang.Math.toRadians(-45))
//                .splineToConstantHeading(new Vector2d(37,-26),java.lang.Math.toRadians(90))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(46,-12), java.lang.Math.toRadians(0))
//                //push first
//                .setTangent(Math.toRadians(90))
//                .lineToY(-59)
//                .setTangent(90)
//                .splineToLinearHeading(new Pose2d(55,-13,Math.toRadians(90)),Math.toRadians(365))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(62,-59, java.lang.Math.toRadians(120)),java.lang.Math.toRadians(300))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(33,-64,Math.toRadians(90)),Math.toRadians(180), new TranslationalVelConstraint(70))
                //close claw after
                .setTangent(Math.toRadians(180))
                .lineToX(31, new TranslationalVelConstraint(70))
                ;

//        TrajectoryActionBuilder act3 = drive.actionBuilder(new Pose2d(37,-26,Math.toRadians(270)))
//
//                ;

        TrajectoryActionBuilder act4 = drive.actionBuilder(new Pose2d(31,-64,Math.toRadians(90)))
                //drop off first
                //move while raising slide
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-1,-32,Math.toRadians(270)),Math.toRadians(90), new TranslationalVelConstraint(70))
                //close claw lower slide
                ;
        TrajectoryActionBuilder act5 = drive.actionBuilder(new Pose2d(-1,-32,Math.toRadians(270)))
                //move back while lowering slide
                .setTangent(Math.toRadians(330))
                .splineToLinearHeading(new Pose2d(33,-64,Math.toRadians(90)),Math.toRadians(180), new TranslationalVelConstraint(70))
                //close claw
                .setTangent(Math.toRadians(180))
                .lineToX(31, new TranslationalVelConstraint(70))
                ;
        TrajectoryActionBuilder act6 = drive.actionBuilder(new Pose2d(31,-64,Math.toRadians(90)))
                //move while raising slide
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(1,-32,Math.toRadians(270)),Math.toRadians(90), new TranslationalVelConstraint(70))
                //lower slide open claw
                ;
        TrajectoryActionBuilder act7 = drive.actionBuilder(new Pose2d(31,-64,Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(4,-32,Math.toRadians(270)),Math.toRadians(90), new TranslationalVelConstraint(70))
                ;
//        TrajectoryActionBuilder act8 = drive.actionBuilder(new Pose2d(4,-32,Math.toRadians(270)))
//                .setTangent(java.lang.Math.toRadians(330))
//                .splineToLinearHeading(new Pose2d(50,-28,java.lang.Math.toRadians(90)), java.lang.Math.toRadians(90),new TranslationalVelConstraint(65))
//                .splineToLinearHeading(new Pose2d(62,-13,java.lang.Math.toRadians(90)), java.lang.Math.toRadians(0), new TranslationalVelConstraint(65))
//                ;
//        TrajectoryActionBuilder act81 = drive.actionBuilder(new Pose2d(65,-13,Math.toRadians(90)))
//                .setTangent(java.lang.Math.toRadians(270))
//                .lineToY(-57)
//                .splineToLinearHeading(new Pose2d(33,-64,java.lang.Math.toRadians(90)),java.lang.Math.toRadians(180), new TranslationalVelConstraint(65))
//                ;
//        TrajectoryActionBuilder act9 = drive.actionBuilder(new Pose2d(35,-64,Math.toRadians(180)))
//                .setTangent(Math.toRadians(120))
//                .splineToLinearHeading(new Pose2d(6,-32,Math.toRadians(270)),Math.toRadians(120), new TranslationalVelConstraint(65))
//                ;
        TrajectoryActionBuilder act10 = drive.actionBuilder(new Pose2d(4,-33,Math.toRadians(120)))
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(63,-63),0, new TranslationalVelConstraint(70))
                ;

//        Action act3 = drive.actionBuilder(new Pose2d(64,-59,Math.toRadians(365)))
//                //collect second
//                .splineToLinearHeading(new Pose2d(35,-63,Math.toRadians(90)),Math.toRadians(180))
//                .setTangent(Math.toRadians(120))
//                //drop off second
//                .splineToLinearHeading(new Pose2d(-3,-33,Math.toRadians(270)),Math.toRadians(120))
//                .setTangent(Math.toRadians(330))
//                //collect third
//                .splineToLinearHeading(new Pose2d(35,-63,Math.toRadians(90)),Math.toRadians(180))
//                .setTangent(Math.toRadians(120))
//                //drop off third
//                .splineToLinearHeading(new Pose2d(0,-33,Math.toRadians(270)),Math.toRadians(120))
//                //park
//                .setTangent(Math.toRadians(270))
//                .splineTo(new Vector2d(65,-65),0)
//                .build();

        Action openClaw = gripper.openGripper();
        Action closeClaw = gripper.closeGripper();
        Action liftUp = lift.LiftUp();
        Action liftMed = lift.LiftMed();
        Action liftDown = lift.LiftDown();

        Actions.runBlocking(closeClaw);

        Action action1 = act1.build();
        Action action2 = act2.build();
        //Action action3 = act3.build();
        Action action4 = act4.build();
        Action action5 = act5.build();
        Action action6 = act6.build();
        Action action7 = act7.build();
//        Action action8 = act8.build();
//        Action action81 = act81.build();
//        Action action9 = act9.build();
        Action action10 = act10.build();


        waitForStart();
        drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Actions.runBlocking(

                new SequentialAction(

                        new ParallelAction(
                                liftUp,
                                action1
                        ),
                        liftMed,
                        openClaw,

                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        liftDown
                                        ),
                                action2
                        ),
                        closeClaw,
                        new SleepAction(0.4),
                        //grab second specimen
                        new ParallelAction(
                                liftUp,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        action4)
                        ),
                        liftMed,
                        openClaw,

//                        new ParallelAction(
//                                lift.liftTo(-22),
//                                action5
//                        ),
//                        closeClaw,
//                        new SleepAction(0.25),
//                        new ParallelAction(
//                                lift.liftTo(-1350),
//                                action6
//                        ),
//                        lift.liftTo(-800),
//                        openClaw,
//                        new SleepAction(0.25),


//        new ParallelAction(
//                lift.liftTo(-22),
//                action5
//        ),
//        closeClaw,
//        new SleepAction(0.25),
//        new ParallelAction(
//                lift.liftTo(-1350),
//                action7
//        ),
//        lift.liftTo(-800),
//        openClaw,
//        new SleepAction(0.25),




//        new ParallelAction(
//                action8,
//                lift.liftTo(-50)
//        ),
//        action81,
//        closeClaw,
//        new ParallelAction(
//                action9,
//                lift.liftTo(-1350)
//        ),
//        lift.liftTo(-800),
//        openClaw,
                        new ParallelAction(
                                action10,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        liftDown
                                        )
                                )
                )
        );
    }
}