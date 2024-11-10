package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "ff")
public class RRAutonContinuityException extends LinearOpMode {
    public static final double FORWARD_1 = 20;
    public static final double STRAFE_LEFT = 20;
    public static final double FORWARD_2 = 10;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_1)
                .strafeLeft(STRAFE_LEFT)
                //.turn(Math.toRadians(90)) //part of sequence builder
                .forward(FORWARD_2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory);
    }

}
