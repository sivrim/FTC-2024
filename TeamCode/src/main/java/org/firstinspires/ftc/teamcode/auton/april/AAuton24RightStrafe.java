package org.firstinspires.ftc.teamcode.auton.april;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.config.ArmUpRightAuton;

@Config
@Autonomous(group = "aaa")
@Disabled
public class AAuton24RightStrafe extends ArmUpRightAuton {
    private ElapsedTime runtime = new ElapsedTime();DcMotor armMotor = null;Servo clawServo;Servo wristServo;DcMotor armMotor2;

    public static double FORWARD_TO_BAR_STEP_2 = 5;
    public static Vector2d STRAFE_DIRECT_STEP_1 = new Vector2d(30, 30);
    public static double BACK_STEP_3 = 5;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM); armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        clawServo.setPosition(MAX_CLAW_CLOSE);wristServo.setPosition(MAX_WRIST_UP);

        Trajectory trajectoryStrafeFromStart = drive.trajectoryBuilder(new Pose2d()).strafeTo(STRAFE_DIRECT_STEP_1).build();
        Trajectory trajectoryForward = drive.trajectoryBuilder(trajectoryStrafeFromStart.end()).forward(FORWARD_TO_BAR_STEP_2).build();
        Trajectory trajectoryGoBack = drive.trajectoryBuilder(trajectoryForward.end()).back(BACK_STEP_3).build();

        waitForStart();if(isStopRequested()) return;

        // arm 1 --> goes down by default                        // arm 2 --> goes back by default

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_UP_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_UP_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);
        wristServo.setPosition(MAX_WRIST_DOWN);

        drive.followTrajectory(trajectoryStrafeFromStart);
        sleep(1000);
        drive.followTrajectory(trajectoryForward);
        sleep(1000);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_DOWN_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor, runtime);
        clawServo.setPosition(MAX_CLAW_OPEN);
        sleep(1000);
        drive.followTrajectory(trajectoryGoBack);

    }

}
