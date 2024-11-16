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
    public static double FORWARD_FROM_START = 18;
    public static double STRAFE_LEFT_GO_TO_BASKET = 8;
    public static double BACK_2 = 15;

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
        wristServo.setPosition(MAX_WRIST_CLOSE);

        Trajectory trajectoryForwardFromStart = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_FROM_START)
                .build();

        Trajectory trajectoryStrafeLeftToBasket = drive.trajectoryBuilder(trajectoryForwardFromStart.end())
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET)
                .build();

        Trajectory trajectoryGoBack = drive.trajectoryBuilder(trajectoryStrafeLeftToBasket.end())
                .back(BACK_2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectoryForwardFromStart);

        drive.followTrajectory(trajectoryStrafeLeftToBasket);

        drive.turn(Math.toRadians(TURN_M45));

        // Reset the motor encoder so that it reads zero ticks
        armMotor2.setPower(ARM2_POWER);
        armMotor.setPower(ARM1_POWER);

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_BACK_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        wristServo.setPosition(MAX_WRIST_OPEN);
        sleep(1000);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_BACK_2_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        sleep(1000);
        wristServo.setPosition(MAX_WRIST_CLOSE);
        sleep(1000);
        clawServo.setPosition(MAX_CLAW_OPEN);
        sleep(1000);
        wristServo.setPosition(MAX_WRIST_OPEN);

        drive.followTrajectory(trajectoryGoBack);

        //        drive.turn(Math.toRadians(TURN_45));
        //        drive.followTrajectory(trajectory3);
        //        drive.followTrajectory(trajectory4);

    }

}
