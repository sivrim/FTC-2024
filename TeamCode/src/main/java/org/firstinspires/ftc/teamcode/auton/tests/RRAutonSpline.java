package org.firstinspires.ftc.teamcode.auton.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Config
@Disabled
@Autonomous(group = "ff")
public class RRAutonSpline extends LinearOpMode {
    public static final double FORWARD_1 = 20;
    public static final double STRAFE_LEFT = 20;
    public static final double FORWARD_2 = 10;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(-45)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj);

    }

}
