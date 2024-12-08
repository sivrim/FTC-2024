package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.config.TurnConstants.TURN_M45;
import static org.firstinspires.ftc.teamcode.config.TurnConstants.TURN_SAMPLE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.config.ArmDown;
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
    public static double STRAFE_RIGHT_GO_TO_SAMPLE2 = 4;
    public static double FORWARD_FROM_DROP = 8;
    public static double BACK_STEP_3 = 16;

    public static double X = 22.5;
    public static double Y = 22.5;

    public static double TURN_RATIO = 1;
    public static double ANGLE = 45;

    public static int SLEEP_TIME = 100;
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;
    SampleMecanumDrive drive;

    //if false, no arm and wrist movement
    public static boolean arm = false;
    //if false, no chassis movement
    public static boolean chassis = true;
    public static boolean april = false;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_START);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajToArmStretch = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_FROM_START_STEP_1)
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET_SAMPLE_1)
                .turn(Math.toRadians(-1  * ANGLE * TURN_RATIO))
                .build();

        TrajectorySequence trajToSample1Drop = drive.trajectorySequenceBuilder(trajToArmStretch.end())
                .back(BACK_STEP_3)
                .build();

        TrajectorySequence trajToSample2Pick = drive.trajectorySequenceBuilder(trajToSample1Drop.end())
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
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .turn(Math.toRadians(-1 * ANGLE * TURN_RATIO))
                .build();


        clawServo.setPosition(MAX_CLAW_CLOSE);
        initAprilTag();

        /////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if(isStopRequested()) return;
        /////////////////////////////////////////////////////////////////////////////////////
        
        clawServo.setPosition(MAX_CLAW_CLOSE);
        //move so we do not drag the arm on floor
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (ARM_1_MOVE_UP_AT_START_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        clawServo.setPosition(MAX_CLAW_CLOSE);

        if(chassis){
            drive.followTrajectorySequence(trajToArmStretch);
        }

        if(arm) {
            moveArmFromStart();
        }
        if(chassis) {
            drive.followTrajectorySequence(trajToSample1Drop);
        }

        if(arm) {
            sleep(300);
            wristServo.setPosition(MAX_WRIST_DOWN);
            sleep(300);
            moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (ARM_1_MOVE_BACK_BASKET_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
            sleep(300);
            wristServo.setPosition(MAX_WRIST_UP);
            sleep(300);
            clawServo.setPosition(MAX_CLAW_OPEN);
        }

        if(chassis) {
            drive.followTrajectorySequence(trajToSample2Pick);
            sleep(100);
            Pose2d currentPose = readAprilTag();
            sleep(300);
            if (april && currentPose != null) {
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

                if(correction.getX() > .5 || correction.getY() > .5 || correction.getHeading() > 3) {
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajToSample2Pick.end())
                            .forward(correction.getX())
                            .strafeRight(correction.getY())
                            .turn(Math.toRadians(correction.getHeading() * TURN_RATIO))
                            .build());
                }
            }
        }

        if(arm) {
            armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int) (ARM_1_SAMPLE_PICK_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
            wristServo.setPosition(MAX_WRIST_DOWN);
            sleep(10);
            clawServo.setPosition(MAX_CLAW_CLOSE);
            moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM_1_SAMPLE_PICK_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);

        }

        if(chassis) {
            drive.followTrajectorySequence(trajToSample2Drop);
        }
    }

    private void moveArmFromStart() {
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        wristServo.setPosition(MAX_WRIST_UP);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM_1_MOVE_BACK_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_2_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
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
