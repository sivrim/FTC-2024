package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "aaa")
public class AAuton24RightPark extends LinearOpMode {
    public static double FORWARD_1 = 3;
    public static double STRAFE_RIGHT = 20;
    private ElapsedTime runtime = new ElapsedTime();

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
                .strafeLeft(STRAFE_RIGHT)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory1);

        drive.followTrajectory(trajectory2);


    }

}
