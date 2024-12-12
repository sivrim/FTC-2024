package org.firstinspires.ftc.teamcode.auton.april;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.config.ArmUp;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(group = "aaa")
@Disabled
public class AprilTagCorrection extends ArmUp {

    SampleMecanumDrive drive;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initAprilTag();

        /////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if(isStopRequested()) return;
        /////////////////////////////////////////////////////////////////////////////////////

            Pose2d currentPose = readAprilTag();
            telemetry.addData("currnet pose, ", currentPose);
            sleep(300);
            if (currentPose != null) {
                Pose2d sample2PickPose;
                if (currentPose.getX() > 0) { //blue
                    sample2PickPose = new Pose2d(54.9, 55.1, -122.5);
                } else { //red
                    sample2PickPose = new Pose2d(-54.9, -55.1, 57.5);
                }
                Pose2d correction = sample2PickPose.minus(currentPose);
                sleep(100);
                printPose2d("desired pose is ", sample2PickPose);
                printPose2d("current pose from april tag isc", currentPose);
                printPose2d("correction is ", correction);
                telemetry.update();

                sleep(10000);

                if(correction.getX() > .5 || correction.getY() > .5 || correction.getHeading() > 3) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
                            .forward(correction.getX())
                            .strafeRight(correction.getY())
                            .turn(Math.toRadians(correction.getHeading() ))
                            .build());
                }
            }

    }

    public Pose2d readAprilTag() {
        visionPortal.resumeStreaming();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        visionPortal.stopStreaming();
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                printAprilTag(detection);
                if(detection.id == 13 || detection.id == 16){
                    return new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                }

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return null;
    }

    private void printAprilTag(AprilTagDetection detection) {
        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
        Position position = detection.robotPose.getPosition();
        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", position.x, position.y, position.z));
        YawPitchRollAngles orientation = detection.robotPose.getOrientation();
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", orientation.getPitch(AngleUnit.DEGREES), orientation.getRoll(AngleUnit.DEGREES), orientation.getYaw(AngleUnit.DEGREES)));
        //telemetry.update();
    }

    private void printPose2d(String message, Pose2d pose) {
        telemetry.addLine(String.format("%s, XYZ %6.1f %6.1f %6.1f  (inch)", message, pose.getX(), pose.getY(), pose.getHeading()));
        //telemetry.update();
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor
                .Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal
                .Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

//        visionPortal.getCameraControl(ExposureControl.class).setExposure()
    }


}
