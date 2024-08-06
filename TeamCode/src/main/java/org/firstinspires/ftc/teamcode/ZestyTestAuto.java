package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;
@Autonomous
public class ZestyTestAuto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "RedCylinder.tflite";
    private static final String[] LABELS = {
            "Red",
    };
    private TfodProcessor tfod;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    private VisionPortal tfodVisionPortal;
    public SampleMecanumDrive sampleMecanumDrive;
    private static final SampleMecanumDrive.DirectionEnum direction = SampleMecanumDrive.DirectionEnum.WEST;
    protected TouchSensor touchSensor = null;
    //    protected ColorSensor colorSensor1 = null;

    protected DcMotor left_slide_motor = null;
    protected DcMotor right_slide_motor = null;
    protected Servo droneLauncher = null;
    protected Servo armServo = null;
    protected Servo claw = null;
    int id1x = 0;
    int id2x = 0;
    int id3x = 0;
    int id4x = 0;
    int id5x = 0;
    int id6x = 0;
    double xvalue = 0;
    int numberDetections = 0;
    double tickratio = 500 / 9.375;
    boolean Position1 = false;
    boolean Position2 = false;
    boolean Position3 = false;
    long Time = 0;

    @Override

    public void runOpMode() {

        initTfod();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap,this.telemetry,direction);
        sampleMecanumDrive.initialzeAttachments(hardwareMap);

        Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence Pos1Place = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(12,-30,Math.toRadians(180)))
                .forward(5)
                .back(5)
                .lineToLinearHeading(new Pose2d(48,-30,0))
                .build();

        TrajectorySequence Pos2Place = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .back(5)
                .lineToLinearHeading(new Pose2d(48,-36,0))
                .build();

        TrajectorySequence Pos3Place = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(12,-30,Math.toRadians(0)))
                .forward(5)
                .back(5)
                .lineToLinearHeading(new Pose2d(12,-38,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(48,-42),0)

                .build();


        waitForStart();

        if (isStopRequested()) return;

        telemetryTfod();

        telemetry.update();

        tfodVisionPortal.close();

        telemetry.update();
        sleep(1000);

        if (Position3 == true) {
            telemetry.addData("pos 3", "");
            //Position = 3
            drive.followTrajectorySequence(Pos3Place);
        }
        else if(Position2 == true){
            telemetry.addData("pos 2","");
            //Position = 2
            drive.followTrajectorySequence(Pos2Place);
        }
        else {
            Position1 = true;
            telemetry.addData("pos 1", "");
            //Position = 1
            drive.followTrajectorySequence(Pos1Place);
        }

        initAprilTag();
        aprilTag.getDetections();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        numberDetections = currentDetections.size();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }



            if (detection.id == 1){
                id1x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if (detection.id == 2){
                id2x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if (detection.id == 3){
                id3x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if(detection.id == 4){
                id4x = (int) ((detection.ftcPose.x * tickratio) + (4.6 * tickratio));
            }
            if (detection.id == 5){
                id5x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if (detection.id == 6){
                id6x = (int) ((detection.ftcPose.x * tickratio) + (4.4 * tickratio));
            }


            // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

        }
        while (numberDetections == 0) {

            aprilTag.getDetections();


            aprilTagDetections();

            Time = Time + 1;
            telemetry.addData("Times repeated:", Time);
            telemetry.update();
            if(Time == 600) {

                break;
            }
        }

        visionPortal.close();

        if(Position1 == true){
            telemetry.addData("move side", id4x);
            telemetry.update();
        }
        else if (Position3 == true) {

            telemetry.addData("move side", id6x);
            telemetry.update();
        }
        else {

        }





        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()


                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder tfodBuilder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            tfodBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            tfodBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        tfodBuilder.enableLiveView(true);
        tfodBuilder.addProcessor(tfod);

        tfodVisionPortal = tfodBuilder.build();
        tfod.setMinResultConfidence(0.8f);
        // Disable or re-enable the TFOD processor at any time.
        tfodVisionPortal.setProcessorEnabled(tfod, true);


    }
    private void aprilTagDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        numberDetections = currentDetections.size();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }


            if (detection.id == 1){
                id1x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if (detection.id == 2){
                id2x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if (detection.id == 3){
                id3x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if(detection.id == 4){
                id4x = (int) ((detection.ftcPose.x * tickratio) + (4.7 * tickratio));
            }
            if (detection.id == 5){
                id5x = (int) ((detection.ftcPose.x * tickratio) + (4.5 * tickratio));
            }
            if (detection.id == 6){
                id6x = (int) ((detection.ftcPose.x * tickratio) + (4.3 * tickratio));
            }

        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            xvalue = recognition.getLeft();

            if (xvalue <= 300) {
                Position1 = false;
                Position2 = true;
                Position3 = false;
                telemetry.addData("Position1 =", Position1);

            } else if (xvalue > 300) {
                Position3 = true;
                Position2 = false;
                Position1 = false;
                telemetry.addData("Position3 =", Position3);
            } else {
                Position2 = false;
                Position1 = true;
                Position3 = false;
                telemetry.addData("Position2 =", Position2);
            }

        }// end for() loop


    }

}
