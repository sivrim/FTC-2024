package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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

    public static double MAX_CLAW_OPEN = 0.6;
    public static double MAX_CLAW_CLOSE = 0.0;
    public static double MAX_WRIST_OPEN = 0.4;
    public static double MAX_WRIST_CLOSE = 0.8;
    public static boolean ENABLE_GAMEPAD_2 = true;


    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure your ID's match your configuration

        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> allDeviceMappings = hardwareMap.allDeviceMappings;

        MacanumWheelsTeleop wheels = new MacanumWheelsTeleop(hardwareMap, telemetry);
        armMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.dcMotor.get(DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.servo.get(DeviceNames.SERVO_CLAW);

        wristServo = hardwareMap.servo.get(DeviceNames.SERVO_WRIST);

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo.setPosition(MAX_WRIST_CLOSE);

        waitForStart();

        if (isStopRequested()) return;

        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo.setPosition(MAX_WRIST_CLOSE);

        while (opModeIsActive()) {
            double chassisY = getChassisY();
            double chassisX = getChassisX();
            double chassisTurn = gamepad1.right_stick_x;

            armMotor.setPower(-1 * gamepad2.left_stick_y);

            float armMotor2Power = -1 * gamepad2.right_stick_y * 1;

//            if(armMotor2.getCurrentPosition() < -8600 && armMotor2Power > 0 ){
//                armMotor2Power = 0;
//            }

            armMotor2.setPower(armMotor2Power);

            setWrist(gamepad1);
            setWrist(gamepad2);

            setClaw(gamepad1);
            setClaw(gamepad2);

            if (gamepad2.x) {
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad2.y) {
                armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            wheels.move(chassisX, chassisY, chassisTurn);

            // Show the position of the motor on telemetry
            telemetry.addData("chassis  power ", chassisX + "" +  chassisY);
            telemetry.addData("gamepad1.left_trigger ", gamepad1.left_trigger);
            telemetry.addData("gamepad1.left_bumper ", gamepad1.left_bumper);

            telemetry.addData("gamepad1.dpad_up ", gamepad1.dpad_up);
            telemetry.addData("gamepad1.dpad_down ", gamepad1.dpad_down);

            telemetry.addData("chassis  power ", chassisX + "" +  chassisY);
            telemetry.addData("gamepad2.left_trigger ", gamepad2.left_trigger);
            telemetry.addData("gamepad2.left_bumper ", gamepad2.left_bumper);

            telemetry.addData("gamepad2.dpad_up ", gamepad2.dpad_up);
            telemetry.addData("gamepad2.dpad_down ", gamepad2.dpad_down);

            telemetry.update();

        }
    }

    private void setWrist(Gamepad gamepad) {
        if(gamepad.left_trigger > 0){
            wristServo.setPosition(MAX_CLAW_CLOSE);
        } else  if(gamepad.left_bumper){
            wristServo.setPosition(MAX_CLAW_OPEN);
        }
    }

    private void setClaw(Gamepad gamepad) {
        if(gamepad.dpad_up){
            clawServo.setPosition(MAX_WRIST_CLOSE);
        } else  if(gamepad.dpad_down) {
            clawServo.setPosition(MAX_WRIST_OPEN);
        } else  if(gamepad.dpad_left) {
            clawServo.setPosition(MAX_WRIST_OPEN + (MAX_WRIST_CLOSE - MAX_WRIST_OPEN) * 0.33);
        }else  if(gamepad.dpad_right) {
            clawServo.setPosition(MAX_WRIST_OPEN+ (MAX_WRIST_CLOSE - MAX_WRIST_OPEN) * 0.66);
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
