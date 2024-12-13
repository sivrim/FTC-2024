package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class AAuton24LeftTrajSequence extends ArmUp {
    public static double FORWARD_FROM_START_STEP_1 = 16;
    public static double STRAFE_LEFT_GO_TO_BASKET_SAMPLE_1 = 7;

    Rev2mDistanceSensor distanceSensor = null;
    public static double STRAFE_RIGHT_GO_TO_SAMPLE2 = 5.0;
    public static double FORWARD_FROM_DROP = 9.5;
    public static double BACK_STEP_3 = 16;
    public static double BACK_STEP_4 = 2.0;

    public static double APRIL_X = 54.9;
    public static double APRIL_Y = 55.7;
    public static double APRIL_ANGLE = 120.5;
    public static double TURN_RATIO = 1;
    public static double ANGLE = 45;

    private ElapsedTime runtime = new ElapsedTime();

    private double ARM_BASKET_POWER = 1.0;

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;
    SampleMecanumDrive drive;

    //if false, no arm and wrist movement
    public static boolean arm = true;
    //if false, no chassis movement
    public static boolean chassis = true;
    public static boolean april = true;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    TrajectorySequence trajToSample1DropAfterDistanceSensor;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_START);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        try{
            distanceSensor = (Rev2mDistanceSensor )(hardwareMap.get(DistanceSensor.class, "2m"));
         } catch (Exception ex){
            ex.printStackTrace();
        }

        TrajectorySequence trajToArmStretch = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_FROM_START_STEP_1)
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET_SAMPLE_1)
                .turn(Math.toRadians(-1  * ANGLE * TURN_RATIO))
                .build();

        TrajectorySequence trajToSample1Drop = drive.trajectorySequenceBuilder(trajToArmStretch.end())
                .back(BACK_STEP_3)
                .build();

        TrajectorySequence trajToSample1DropAfterDistanceSensor = drive.trajectorySequenceBuilder(trajToSample1Drop.end())
                .back(BACK_STEP_4)
                .build();

        TrajectorySequence trajToSample2Pick = drive.trajectorySequenceBuilder(trajToSample1DropAfterDistanceSensor.end())
                .forward(FORWARD_FROM_DROP)
                .turn(Math.toRadians(ANGLE * TURN_RATIO))
                .strafeRight(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .build();

        TrajectorySequence trajToSample2Drop = drive.trajectorySequenceBuilder(trajToSample2Pick.end())
                .turn(Math.toRadians(-1 * ANGLE * TURN_RATIO))
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .back(FORWARD_FROM_DROP)
                .build();

        TrajectorySequence trajToSample3Pick = drive.trajectorySequenceBuilder(trajToSample2Drop.end())
                .forward(FORWARD_FROM_DROP)
                .turn(Math.toRadians(ANGLE * TURN_RATIO))
                .strafeRight(STRAFE_RIGHT_GO_TO_SAMPLE2 -10)
                .build();

        TrajectorySequence trajToSample3Drop = drive.trajectorySequenceBuilder(trajToSample3Pick.end())
                .strafeRight(10)
                .turn(Math.toRadians(-1 * ANGLE * TURN_RATIO))
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .back(FORWARD_FROM_DROP)
                .build();

        clawServo.setPosition(MAX_CLAW_CLOSE);
        initAprilTag();

        telemetry.addLine("Initialaztion complete");
        telemetry.update();
        /////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if(isStopRequested()) return;
        /////////////////////////////////////////////////////////////////////////////////////
        telemetry.addLine("Starting");
        telemetry.update();
        clawServo.setPosition(MAX_CLAW_CLOSE);
        //move so we do not drag the arm on floor
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (SAMPLE_1_ANGLE_ARM_1_AWAY_FROM_FLOOR * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        telemetry.addLine("Arm moved off floor");
        telemetry.update();
        clawServo.setPosition(MAX_CLAW_CLOSE);

        dropSample1(drive, trajToArmStretch, trajToSample1Drop);

        pickSample2(drive, trajToSample2Pick);

        sleep(50);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_2_ARM_1_DROP_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        drive.followTrajectorySequence(trajToSample2Drop);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_2_ARM_1_DROP_ANGLE_2 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime, distanceSensor, ARM_BASKET_POWER);

        sleep(800);
        wristServo.setPosition(MAX_WRIST_UP);

        telemetry.addLine("wrist open ");

        sleep(350);
        clawServo.setPosition(MAX_CLAW_OPEN);
        sleep(500);
        drive.followTrajectorySequence(trajToSample3Pick);
//        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int) (SAMPLE_2_ARM_1_SAMPLE_PICK_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
//        wristServo.setPosition(MAX_WRIST_DOWN);
    }

    private void pickSample2(SampleMecanumDrive drive, TrajectorySequence trajToSample2Pick) {
        drive.followTrajectorySequence(trajToSample2Pick);

        Pose2d currentPose = null;
        if(april) {
            sleep(100);
            currentPose = readAprilTag();
            sleep(200);
        }
        if (april && currentPose != null) {
            Pose2d sample2PickPose;
            if (currentPose.getX() > 0) { //blue
                sample2PickPose = new Pose2d(54.9, 55.7, -120.5);
            } else { //red
                sample2PickPose = new Pose2d(-53.9, -55, 60);
            }

            Pose2d correction = sample2PickPose.minus(currentPose);
            printPose2d("desired pose is ", sample2PickPose);
            printPose2d("current pose from april tag isc", currentPose);

            printPose2d("correction is ", correction);
            telemetry.update();

            if(Math.abs(correction.getX()) > .5 || Math.abs(correction.getY()) > .5 || Math.abs(correction.getHeading()) > 3) {
                printPose2d("applying correction", correction);

                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajToSample2Pick.end())
                        .forward(correction.getX())
                        .strafeRight(correction.getY())
                        .turn(Math.toRadians(correction.getHeading() * TURN_RATIO))
                        .build());
            } else {
                printPose2d("NOTTTTTTTTTTTTTTTTT applying correction", correction);
            }
        }

        sleep(50);
        wristServo.setPosition(MAX_WRIST_DOWN);
        sleep(50);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int) (SAMPLE_2_ARM_1_SAMPLE_PICK_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        sleep(250);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        sleep(500);

    }

    private void dropSample1(SampleMecanumDrive drive, TrajectorySequence trajToArmStretch, TrajectorySequence trajToSample1Drop) {

        drive.followTrajectorySequence(trajToArmStretch);
        moveArmFromStart();
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.followTrajectorySequence(trajToSample1Drop);

        sleep(100);
        wristServo.setPosition(MAX_WRIST_DOWN);
        sleep(100);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_2 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime, distanceSensor, ARM_BASKET_POWER);
        drive.followTrajectorySequence(trajToSample1DropAfterDistanceSensor);
        sleep(2000);
        wristServo.setPosition(MAX_WRIST_UP);
        sleep(300);
        clawServo.setPosition(MAX_CLAW_OPEN);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void moveArmFromStart() {
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_1 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        wristServo.setPosition(MAX_WRIST_UP);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_2 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
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
    }


}
