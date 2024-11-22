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

@Config
@Autonomous(group = "aaa")
public class AAuton24Left extends ArmUp {
    public static double FORWARD_FROM_START_STEP_1 = 25;
    public static double STRAFE_LEFT_GO_TO_BASKET_STEP_2 = 18;
    public static double BACK_STEP_3 = 19;

    public static double FORWARD_TO_START_STEP_4 = 20;
    public static double STRAFE_TO_PARK_STEP_5 = 60;

//    public static double PARK_STEP_5 = 50;

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

        Trajectory trajectoryStrafeLeftToBasket = drive.trajectoryBuilder(trajectoryForwardFromStart.end())
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET_STEP_2)
                .build();

        Trajectory trajectoryGoBack = drive.trajectoryBuilder(trajectoryStrafeLeftToBasket.end())
                .back(BACK_STEP_3)
                .build();

        Trajectory trajectoryForwardToStart = drive.trajectoryBuilder(new Pose2d()) //(trajectoryGoBack.end())
                .forward(FORWARD_TO_START_STEP_4)
                .build();

        Trajectory trajectoryPark = drive.trajectoryBuilder(trajectoryForwardToStart.end())
                .forward(STRAFE_TO_PARK_STEP_5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(10 * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);


        drive.followTrajectory(trajectoryForwardFromStart);

        /**
         * arm 1 --> goes down by default
         * arm 2 --> goes back by default
         */

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_2_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);

        wristServo.setPosition(MAX_WRIST_DROP);

        drive.followTrajectory(trajectoryStrafeLeftToBasket);
        sleep(1000);
        drive.turn(Math.toRadians(TURN_M50));
        sleep(1000);
        drive.followTrajectory(trajectoryGoBack);
        sleep(1000);
        clawServo.setPosition(MAX_CLAW_OPEN);
        sleep(1000);
        wristServo.setPosition(MAX_WRIST_DOWN);
        sleep(1000);
//
//        drive.turn(Math.toRadians(TURN_PARK));
//        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM_1_PARK_ANGLE_1 * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);
//        drive.followTrajectory(trajectoryForwardToStart);
//        drive.followTrajectory(trajectoryPark);
//        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM_1_PARK_ANGLE_2 * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);


    }

}
