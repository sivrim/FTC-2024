package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "ff")
public class RRAutonMultipleTrajectories extends LinearOpMode {
    public static double FORWARD_1 = 6;
    public static double STRAFE_LEFT = 18;
    public static double TURN_90 = 90;
    public static double TURN_135 = 135;
    public static double TURN_M45 = -45;
    public static double TURN_45 = 45;

    public static int ARM_1_MOVE_BACK = 1000;
    public static double ARM1_POWER = 1.0;

    public static final double FORWARD_2 = 6;

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

        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(STRAFE_LEFT)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .forward(FORWARD_2)
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .back(FORWARD_2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory1);

        drive.followTrajectory(trajectory2);

        drive.turn(Math.toRadians(TURN_M45));

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Arm motor position at very start", armMotor.getCurrentPosition());

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(ARM1_POWER);
        armMotor.setTargetPosition(ARM_1_MOVE_BACK);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        int timeout = 5;
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (armMotor.isBusy() )) {
            // Display it for the driver.
            telemetry.addData("Running to",  " %7d ", ARM_1_MOVE_BACK);
            telemetry.addData("Currently at",  " at %7d ",
                    armMotor.getCurrentPosition());
            telemetry.update();
        }

        sleep(1000);
        telemetry.addData("After motion, Currently at",  " at %7d ",
                armMotor.getCurrentPosition());
        telemetry.update();

//        drive.turn(Math.toRadians(TURN_45));
//
//        drive.followTrajectory(trajectory3);
//
//        drive.followTrajectory(trajectory4);
//
//
//        drive.turn(Math.toRadians(TURN_M45));
//
//        drive.turn(Math.toRadians(TURN_45));
//
//        drive.followTrajectory(trajectory3);
//
//        drive.followTrajectory(trajectory4);
    }

}
