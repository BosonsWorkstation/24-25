package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@Autonomous
public class moveAuto extends LinearOpMode {

    private MecanumDrive mecanumDrive;

    public class Lift {
        private DcMotorEx lift1;
        private DcMotorEx lift2;

        public Lift(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "rightSlide");
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2 = hardwareMap.get(DcMotorEx.class, "leftSlide");
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }




        public Action liftTo(int distance, double power){
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    int lift1Pos = lift1.getCurrentPosition();
                    int lift2Pos = lift2.getCurrentPosition();

                    lift1.setTargetPosition(distance);
                    lift2.setTargetPosition(distance);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while(opModeIsActive()) {
                        lift1.setPower(power);
                        lift2.setPower(power);
                        if(lift1Pos >= distance - 20 || lift1Pos <= distance + 20){
                            break;
                        }
                    }

                    lift1.setPower(0);
                    lift2.setPower(0);
                    return false;



                }
            };
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

        TrajectoryActionBuilder act1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .lineToX(63);



        Action endTraj = act1.endTrajectory().fresh()
                .build();
        Action openClaw = gripper.openGripper();
        Action closeClaw = gripper.closeGripper();
        Actions.runBlocking(closeClaw);


        waitForStart();

        Action action1 = act1.build();
        Actions.runBlocking(


                new SequentialAction(
                        action1
                        )







        );




    }


}
