package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.config.ArmUp;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "aaa")
@Disabled
public class AAutonFoldedArms extends ArmUp {
    public static double SAMPLE_1_DROP_FORWARD_FROM_START_STEP_1 = 16;
    public static double SAMPLE_1_DROP_LEFT_GO_TO_BASKET_STEP_2 = 7;
    public static double STRAFE_RIGHT_GO_TO_SAMPLE2 = 4.5;
    public static double FORWARD_FROM_DROP = 7;
    public static double SAMPLE_1_DROP_BACK_TO_BASKET_STEP_3 = 15.5;

    public static double TURN_RATIO = 1;
    public static double ANGLE_45 = 45;
    public static int SLEEP_DROP = 2000;

    public static double ARM_MOTOR_2_POWER = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;Servo clawServo;Servo wristServo;DcMotor armMotor2;SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        initMotors();

        TrajectorySequence trajToSample1Drop = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(SAMPLE_1_DROP_FORWARD_FROM_START_STEP_1)
                .strafeLeft(SAMPLE_1_DROP_LEFT_GO_TO_BASKET_STEP_2)
                .turn(Math.toRadians(-1  * ANGLE_45 * TURN_RATIO))
                .back(SAMPLE_1_DROP_BACK_TO_BASKET_STEP_3)
                .build();

        TrajectorySequence trajToSample2Pick = drive.trajectorySequenceBuilder(trajToSample1Drop.end())
                .forward(FORWARD_FROM_DROP)
                .turn(Math.toRadians(ANGLE_45 * TURN_RATIO))
                .strafeRight(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .build();

        TrajectorySequence trajToSample2Drop = drive.trajectorySequenceBuilder(trajToSample2Pick.end())
                .turn(Math.toRadians(-1 * ANGLE_45 * TURN_RATIO))
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .back(FORWARD_FROM_DROP)
                .build();

        TrajectorySequence trajToSample3Pick = drive.trajectorySequenceBuilder(trajToSample2Drop.end())
                .forward(FORWARD_FROM_DROP)
                .turn(Math.toRadians(ANGLE_45 * TURN_RATIO))
                .strafeRight(STRAFE_RIGHT_GO_TO_SAMPLE2 -10)
                .build();

        TrajectorySequence trajToSample3Drop = drive.trajectorySequenceBuilder(trajToSample3Pick.end())
                .strafeRight(10)
                .turn(Math.toRadians(-1 * ANGLE_45 * TURN_RATIO))
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .back(FORWARD_FROM_DROP)
                .build();

        clawServo.setPosition(MAX_CLAW_CLOSE);

        /////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if(isStopRequested()) return;
        /////////////////////////////////////////////////////////////////////////////////////

        clawServo.setPosition(MAX_CLAW_CLOSE);
        //move so we do not drag the arm on floor
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (SAMPLE_1_ANGLE_ARM_1_AWAY_FROM_FLOOR * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        clawServo.setPosition(MAX_CLAW_CLOSE);

        dropSample1(drive, trajToSample1Drop);

            drive.followTrajectorySequence(trajToSample2Pick);

            moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int) (SAMPLE_2_ARM_1_SAMPLE_PICK_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
            sleep(200);
            wristServo.setPosition(MAX_WRIST_DOWN);
            sleep(500);
            clawServo.setPosition(MAX_CLAW_CLOSE);
            sleep(500);
            moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_2_ARM_1_DROP_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
            sleep(500);


            drive.followTrajectorySequence(trajToSample2Drop);

            drive.followTrajectorySequence(trajToSample3Pick);

            drive.followTrajectorySequence(trajToSample3Drop);

    }

    private void initMotors() {
        drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_START);
    }

    private void dropSample1(SampleMecanumDrive drive, TrajectorySequence trajectorySeq) {
        drive.followTrajectorySequence(trajectorySeq);

        stretchArms();

        sleep(300);
        wristServo.setPosition(MAX_WRIST_DOWN);
        sleep(300);
        armMotor.setPower(0.5);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_2 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        sleep(SLEEP_DROP);
        wristServo.setPosition(MAX_WRIST_UP);
        sleep(300);
        clawServo.setPosition(MAX_CLAW_OPEN);
        armMotor.setPower(1.0);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static int DROP_ARM2_ANGLE_1 = 90;
    public static int DROP_ARM1_ANGLE_1 = 90;
    public static int DROP_ARM2_ANGLE_2 = 45;
    public static int DROP_ARM1_ANGLE_2 = 45;
    private void stretchArms() {

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_1 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        wristServo.setPosition(MAX_WRIST_UP);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_2 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);

//        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(DROP_ARM2_ANGLE_1 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
//        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(DROP_ARM1_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
//
//        armMotor2.setPower(1);
//        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(DROP_ARM2_ANGLE_2 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
//        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(DROP_ARM1_ANGLE_2 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
//        wristServo.setPosition(MAX_WRIST_DOWN);
    }

    private void foldArms() {
        wristServo.setPosition(MAX_WRIST_DOWN);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(DROP_ARM2_ANGLE_1 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(DROP_ARM1_ANGLE_1 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);

        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(DROP_ARM2_ANGLE_2 * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(DROP_ARM1_ANGLE_2 * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
    }

}
