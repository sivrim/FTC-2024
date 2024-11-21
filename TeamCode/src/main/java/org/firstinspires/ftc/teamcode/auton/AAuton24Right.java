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
import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.config.ArmUp;
import org.firstinspires.ftc.teamcode.config.ArmUpRightAuton;

@Config
@Autonomous(group = "aaa")
public class AAuton24Right extends ArmUpRightAuton {
    public static double FORWARD_FROM_START_STEP_1 = 4;
    public static double STRAFE_LEFT_STRAFE_STEP_2 = 18;
    public static double FORWARD_TO_BAR_3 = 30;

    public static double FORWARD_TO_START_STEP_4 = 5;

    public static double STRAFE_TO_BASKET_STEP_5 = 50;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_UP);

        Trajectory trajectoryForwardFromStart = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_FROM_START_STEP_1)
                .build();

        Trajectory trajectoryStrafeLeft = drive.trajectoryBuilder(trajectoryForwardFromStart.end())
                .strafeLeft(STRAFE_LEFT_STRAFE_STEP_2)
                .build();

        Trajectory trajectoryForwardToBar = drive.trajectoryBuilder(trajectoryStrafeLeft.end())
                .back(FORWARD_TO_BAR_3)
                .build();

        Trajectory trajectoryGoBack = drive.trajectoryBuilder(trajectoryForwardToBar.end())
                .forward(FORWARD_TO_START_STEP_4)
                .build();

//        Trajectory trajectoryStrafeLeftToPark = drive.trajectoryBuilder(trajectoryGoBack.end())
//                .strafeLeft(STRAFE_TO_BASKET_STEP_5)
//                .build();

        waitForStart();

        if(isStopRequested()) return;

        /**
         * arm 1 --> goes down by default
         * arm 2 --> goes back by default
         */

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_UP_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_UP_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);

        wristServo.setPosition(MAX_WRIST_DROP);

        drive.followTrajectory(trajectoryForwardFromStart);
        drive.followTrajectory(trajectoryStrafeLeft);
        sleep(200);
        drive.followTrajectory(trajectoryForwardToBar);
        sleep(200);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_DOWN_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);
        sleep(500);

        clawServo.setPosition(MAX_CLAW_OPEN);
        drive.followTrajectory(trajectoryGoBack);

    }

}
