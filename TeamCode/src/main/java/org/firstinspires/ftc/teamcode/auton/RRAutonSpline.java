package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;

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
                .splineTo(new Vector2d(FORWARD_1, 0), Math.toRadians(0))
                .splineTo(new Vector2d(FORWARD_1, STRAFE_LEFT), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj);

    }

}
