package org.firstinspires.ftc.teamcode.auton.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "aaa")
@Disabled
public class AAuton24LeftPark extends LinearOpMode {
    public static double FORWARD_1 = 3;
    public static double STRAFE_LEFT = 22;
    private ElapsedTime runtime = new ElapsedTime();

    public static double MAX_CLAW_OPEN = 0.1;
    public static double MAX_CLAW_CLOSE = 1.0;
    public static double MAX_WRIST_OPEN = 0.0;
    public static double MAX_WRIST_CLOSE = 1.0;
    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(STRAFE_LEFT)
                .build();
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);

        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo.setPosition(MAX_CLAW_CLOSE);
        waitForStart();
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo.setPosition(MAX_CLAW_CLOSE);


        if(isStopRequested()) return;

        drive.followTrajectory(trajectory1);

        drive.followTrajectory(trajectory2);


    }

}
