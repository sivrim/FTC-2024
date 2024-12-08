package org.firstinspires.ftc.teamcode.auton.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Config
@Disabled
@Autonomous(group = "ff")
public class RRAutonTurn extends LinearOpMode {
    public static final double FORWARD_1 = 20;
    public static final double STRAFE_LEFT = 20;
    public static final double TURN_135 = 135;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(STRAFE_LEFT)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(STRAFE_LEFT)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory1);
        drive.followTrajectory(trajectory2);
        drive.turn(Math.toRadians(TURN_135));

    }

}
