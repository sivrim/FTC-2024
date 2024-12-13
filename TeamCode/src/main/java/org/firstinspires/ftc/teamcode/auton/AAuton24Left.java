package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.config.ArmDown;
import org.firstinspires.ftc.teamcode.config.ArmUp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.config.TurnConstants.*;

import java.util.List;

@Config
@Disabled
@Autonomous(group = "aaa")
public class AAuton24Left extends ArmUp {
    public static double FORWARD_FROM_START_STEP_1 = 25;
    public static double STRAFE_LEFT_GO_TO_BASKET_STEP_2 = 18;
    public static double BACK_STEP_3 = 18;

    public static double TURN_RATIO = 1.5;
    public static double FORWARD_TO_START_STEP_4 = 1;
    public static double STRAFE_TO_PARK_STEP_5 = 60;

//    public static double PARK_STEP_5 = 50;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;
    SampleMecanumDrive drive;

    Trajectory lastTrajectory;
    Trajectory sample1_trajectoryForwardFromStart, sample1_trajectoryStrafeLeftToBasket, sample1_trajectoryGoBack;
    Trajectory trajectoryForwardToStart, trajectoryForwardToTurn;
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
        wristServo.setPosition(MAX_WRIST_UP);

        /**
         * arm 1 --> goes down on forward
         * arm 2 --> goes up on forward
         */

        //do this on init, before start, as each trajectory building can take up to .5 seconds.
        initTrajectories();
        initAprilTag();
        waitForStart();

        if(isStopRequested()) return;

        //List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        dropSample1();

        sleep(2000);
        drive.followTrajectory(trajectoryForwardToStart);
        sleep(2000);

        drive.turn(Math.toRadians(TURN_SAMPLE * TURN_RATIO));

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ArmDown.ARM_1_MOVE_DOWN_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        clawServo.setPosition(MAX_WRIST_DOWN);

    }

    public void moveReadCorrect(Pose2d start, Pose2d end) {
        move(start, end);
        Pose2d currentPose = readAprilTag();
        if(currentPose != null) {
            //TODO add leniency
            boolean withinMarginOfError = Math.abs(currentPose.getX() - end.getX()) < .2
                    && Math.abs(currentPose.getY() - end.getY()) < .2
                    && Math.abs(currentPose.getHeading() - end.getHeading()) < Math.toRadians(2);

            if(!withinMarginOfError) {
                move(currentPose, end);
            }
        }
    }

    public void move(Pose2d start, Pose2d end) {
        Pose2d startOfTrajectory = lastTrajectory == null ? new Pose2d() : lastTrajectory.end();

        lastTrajectory = drive.trajectoryBuilder(startOfTrajectory)
                .lineToLinearHeading(new Pose2d(end.getX() - start.getX(), end.getY() - start.getY(), end.getHeading() - start.getHeading()))
                .build();

        drive.followTrajectory(lastTrajectory);

    }

    public Pose2d readAprilTag() {
        visionPortal.resumeStreaming();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        visionPortal.stopStreaming();
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                Position position = detection.robotPose.getPosition();
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        position.x,
                        position.y,
                        position.z));
                YawPitchRollAngles orientation = detection.robotPose.getOrientation();
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        orientation.getPitch(AngleUnit.DEGREES),
                        orientation.getRoll(AngleUnit.DEGREES),
                        orientation.getYaw(AngleUnit.DEGREES)));
                if(detection.id == 13 || detection.id == 16){
                    return new Pose2d(position.x, position.y, orientation.getYaw());
                }

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return null;
    }


    private void dropSample1() {
        //move arm up so it does not drag against surface when moving
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(10 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);

        drive.followTrajectory(sample1_trajectoryForwardFromStart);

        //move arm up, while we are away from all walls and basket. Otherwise, moving arms has to be co-ordinated
        moveArmFromStart();

        wristServo.setPosition(MAX_WRIST_DOWN);
        drive.followTrajectory(sample1_trajectoryStrafeLeftToBasket);
        drive.turn(Math.toRadians(TURN_M45 * TURN_RATIO));
        drive.followTrajectory(sample1_trajectoryGoBack);
        clawServo.setPosition(MAX_CLAW_OPEN);
        wristServo.setPosition(MAX_WRIST_DOWN);
    }

    private void moveArmFromStart() {
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_1 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_2 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
    }

    private void initTrajectories() {
        sample1_trajectoryForwardFromStart = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_FROM_START_STEP_1)
                .build();

        sample1_trajectoryStrafeLeftToBasket = drive.trajectoryBuilder(sample1_trajectoryForwardFromStart.end())
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET_STEP_2)
                .build();

        sample1_trajectoryGoBack = drive.trajectoryBuilder(sample1_trajectoryStrafeLeftToBasket.end())
                .back(BACK_STEP_3)
                .build();

        trajectoryForwardToStart = drive.trajectoryBuilder(sample1_trajectoryGoBack.end())
                .forward(FORWARD_TO_START_STEP_4)
                .build();

        trajectoryForwardToTurn = drive.trajectoryBuilder(trajectoryForwardToStart.end())
                .forward(STRAFE_TO_PARK_STEP_5)
                .build();


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
