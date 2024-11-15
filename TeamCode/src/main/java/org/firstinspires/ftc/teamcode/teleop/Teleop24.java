package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;

import java.util.List;

/**
 * Second chassis only has 4 dc motors. No arm motor or any other servo.
 */
@TeleOp(name = "AATeleop", group = "aaa")
@Config
public class Teleop24 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;

    public static double MAX_CLAW_OPEN = 0.25;
    public static double MAX_CLAW_CLOSE = 0.8;
    public static double MAX_WRIST_OPEN = .15;
    public static double MAX_WRIST_CLOSE = 0.75;


    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure your ID's match your configuration

        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> allDeviceMappings = hardwareMap.allDeviceMappings;

        allDeviceMappings.forEach(d -> {
            System.out.println(d.getDeviceTypeClass().getCanonicalName());
        });

        MacanumWheelsTeleop wheels = new MacanumWheelsTeleop(hardwareMap, telemetry);
        armMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.dcMotor.get(DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.servo.get(DeviceNames.SERVO_CLAW);

        wristServo = hardwareMap.servo.get(DeviceNames.SERVO_WRIST);

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reset the motor encoder so that it reads zero ticks
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        clawServo.setPosition(MAX_CLAW_OPEN);
        wristServo.setPosition(MAX_WRIST_OPEN);

        while (opModeIsActive()) {
            double chassisY = getChassisY();
            double chassisX = getChassisX();
            double chassisTurn = gamepad1.right_stick_x;

            // Show the position of the motor on telemetry
            telemetry.addData("arm motor 1 ", armMotor.getCurrentPosition());
            telemetry.addData("arm motor 2 ", armMotor2.getCurrentPosition());
            telemetry.update();

            armMotor.setPower(gamepad2.left_stick_y);
            armMotor2.setPower(-1 * gamepad2.right_stick_y);

            if(gamepad2.left_trigger > 0){
                clawServo.setPosition(MAX_CLAW_CLOSE);
            } else  if(gamepad2.left_bumper){
                clawServo.setPosition(MAX_CLAW_OPEN);
            }

            /**
             *                                                              Wrist
             */

            if(gamepad2.dpad_up){
                wristServo.setPosition(MAX_WRIST_CLOSE);
            } else  if(gamepad2.dpad_down){
                wristServo.setPosition(MAX_WRIST_OPEN);
            }


            if(gamepad1.x){
                wristServo.setPosition(MAX_WRIST_CLOSE);
            } else  if(gamepad1.b){
                wristServo.setPosition(MAX_WRIST_OPEN);
            }

            if (gamepad2.x) {
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            wheels.move(chassisX, chassisY, chassisTurn);
            telemetry.update();

        }
    }

    private double getChassisX() {
        double chassisX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        //  System.out.println("gamepad1.left_stick_x is " + chassisX);
        return chassisX;
    }

    private double getChassisY() {
        double chassisY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        //  System.out.println("gamepad1.left_stick_y is " + chassisY);
        return chassisY;
    }
}
