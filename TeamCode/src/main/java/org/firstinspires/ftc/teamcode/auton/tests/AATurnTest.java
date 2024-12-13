package org.firstinspires.ftc.teamcode.auton.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.config.ArmUp;
import static org.firstinspires.ftc.teamcode.config.TurnConstants.*;
@Config
@Disabled
@Autonomous(group = "aaa")
public class AATurnTest extends ArmUp {
    public static double FORWARD_FROM_START_STEP_1 = 25;

    public static double TURN_RATIO = 1.5;
    public static double STRAFE_LEFT_GO_TO_BASKET_STEP_2 = 18;
    public static double BACK_STEP_3 = 19;

    public static double FORWARD_TO_START_STEP_4 = 20;
    public static double STRAFE_TO_PARK_STEP_5 = 60;

//    public static double PARK_STEP_5 = 50;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

//        drive.followTrajectory(trajectoryForwardFromStart);
//
//        drive.followTrajectory(trajectoryStrafeLeftToBasket);
        drive.turn(Math.toRadians(TURN_M45 * TURN_RATIO));
        sleep(1000);
//        drive.followTrajectory(trajectoryGoBack);
        sleep(1000);
        drive.turn(Math.toRadians(TURN_45 * TURN_RATIO));
        sleep(1000);
        drive.turn(Math.toRadians(TURN_M90 * TURN_RATIO));
        sleep(1000);
        drive.turn(Math.toRadians(TURN_90 * TURN_RATIO));
        sleep(1000);

    }

}
