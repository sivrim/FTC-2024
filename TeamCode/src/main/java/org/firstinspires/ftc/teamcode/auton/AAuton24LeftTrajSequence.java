package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.config.TurnConstants.TURN_M45;
import static org.firstinspires.ftc.teamcode.config.TurnConstants.TURN_SAMPLE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    public static double FORWARD_FROM_START_STEP_1 = 25;
    public static double STRAFE_LEFT_GO_TO_BASKET_STEP_2 = 18;
    public static double BACK_STEP_3 = 18;

    public static double X = 22.5;
    public static double Y = 22.5;

    public static double TURN_RATIO = 1.5;
    public static double ANGLE = 45;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_UP);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(FORWARD_FROM_START_STEP_1)
                .addDisplacementMarker(5, () -> moveArmFromStart())
                //.lineToLinearHeading(new Pose2d(BACK_STEP_3, STRAFE_LEFT_GO_TO_BASKET_STEP_2))
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET_STEP_2)
                .turn(Math.toRadians(ANGLE * TURN_RATIO))
                .back(BACK_STEP_3)
                .build();

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .forward(22.5)
                .build();

        waitForStart();
        if(isStopRequested()) return;

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(10 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);

        drive.followTrajectorySequence(trajSeq);
        //drive.followTrajectory(traj);
    }

    private void moveArmFromStart() {
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM_1_MOVE_BACK_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_2_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
    }

}
