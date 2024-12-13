package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.config.ArmUpRightAuton;

@Config
@Autonomous(group = "aaa")
public class AAuton24Right extends ArmUpRightAuton {
    private ElapsedTime runtime = new ElapsedTime();DcMotor armMotor = null;Servo clawServo;Servo wristServo;DcMotor armMotor2;

    public static double FORWARD_FROM_START_STEP_1 = 4;
    public static double STRAFE_LEFT_STRAFE_STEP_2 = 18;
    public static double FORWARD_TO_BAR_3 = 25.5;
    public static double BACK_STEP_4 = 28;
    public static double STRAFE_RIGHT_STEP_5 = 60;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM); armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo.setPosition(MAX_WRIST_START);

        Trajectory trajectoryForwardFromStart = drive.trajectoryBuilder(new Pose2d()).forward(FORWARD_FROM_START_STEP_1).build();
        Trajectory trajectoryStrafeLeft = drive.trajectoryBuilder(trajectoryForwardFromStart.end()).strafeLeft(STRAFE_LEFT_STRAFE_STEP_2).build();
        Trajectory trajectoryForwardToBar = drive.trajectoryBuilder(trajectoryStrafeLeft.end()).forward(FORWARD_TO_BAR_3).build();
        Trajectory trajectoryGoBack = drive.trajectoryBuilder(trajectoryForwardToBar.end()).back(BACK_STEP_4).build();
        Trajectory trajectoryPark = drive.trajectoryBuilder(trajectoryGoBack.end()).strafeRight(STRAFE_RIGHT_STEP_5).build();

        waitForStart();if(isStopRequested()) return;

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_UP_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM_1_MOVE_UP_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);
        wristServo.setPosition(MAX_WRIST_DOWN);

        sleep(500);

        drive.followTrajectory(trajectoryForwardFromStart);
        drive.followTrajectory(trajectoryStrafeLeft);
        sleep(500);
        drive.followTrajectory(trajectoryForwardToBar);
        sleep(500);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_DOWN_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);
        sleep(500);
        clawServo.setPosition(MAX_CLAW_OPEN);
        sleep(500);
        drive.followTrajectory(trajectoryGoBack);

        sleep(500);
        drive.followTrajectory(trajectoryPark);

    }

}
