package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    TouchSensor touchSensor = null;

    DcMotor armMotor = null; Servo clawServo; Servo wristServo; DcMotor armMotor2;

    public static double MAX_CLAW_OPEN = 0.6;
    public static double MAX_CLAW_CLOSE = 0.0;
    public static double MAX_WRIST_OPEN = 0.4;
    public static double MAX_WRIST_CLOSE = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure your ID's match your configuration

        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> allDeviceMappings = hardwareMap.allDeviceMappings;

        MacanumWheelsTeleop wheels = new MacanumWheelsTeleop(hardwareMap, telemetry);
        armMotor = hardwareMap.dcMotor.get(DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.dcMotor.get(DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.servo.get(DeviceNames.SERVO_CLAW);

        wristServo = hardwareMap.servo.get(DeviceNames.SERVO_WRIST);

        try{
            touchSensor = hardwareMap.get(TouchSensor.class, "mag");
        }catch (Exception ex){
            ex.printStackTrace();
        }


        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        clawServo.setPosition(MAX_CLAW_CLOSE);
//        wristServo.setPosition(MAX_WRIST_CLOSE);

        waitForStart();

        if (isStopRequested()) return;

//        clawServo.setPosition(MAX_CLAW_CLOSE);
//        wristServo.setPosition(MAX_WRIST_CLOSE);

        while (opModeIsActive()) {
            double chassisY = -gamepad1.left_stick_y;
            double chassisX = gamepad1.left_stick_x * 1.1;
            double chassisTurn = gamepad1.right_stick_x;

            if(touchSensor != null && touchSensor.isPressed() && -gamepad2.left_stick_y < 0) {
                armMotor.setPower(0);
            } else {
                armMotor.setPower(-1 * gamepad2.left_stick_y);
            }

            float armMotor2Power = -1 * gamepad2.right_stick_y * .8f;

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
//            telemetry.addData("chassis  power ", chassisX + "" +  chassisY);
//            telemetry.addData("gamepad1.left_trigger ", gamepad1.left_trigger);
//            telemetry.addData("gamepad1.left_bumper ", gamepad1.left_bumper);
//
//            telemetry.addData("gamepad1.dpad_up ", gamepad1.dpad_up);
//            telemetry.addData("gamepad1.dpad_down ", gamepad1.dpad_down);
//
//            telemetry.addData("chassis  power ", chassisX + "" +  chassisY);
//            telemetry.addData("gamepad2.left_trigger ", gamepad2.left_trigger);
//            telemetry.addData("gamepad2.left_bumper ", gamepad2.left_bumper);
//
//            telemetry.addData("gamepad2.dpad_up ", gamepad2.dpad_up);
//            telemetry.addData("gamepad2.dpad_down ", gamepad2.dpad_down);

            if(touchSensor != null){
                telemetry.addData("magnetic distance ", touchSensor.isPressed());
            }

            telemetry.update();

        }
    }

    private void setWrist(Gamepad gamepad) {
        if(gamepad.left_trigger > 0){
            wristServo.setPosition(MAX_WRIST_CLOSE);
        } else  if(gamepad.left_bumper){
            wristServo.setPosition(MAX_WRIST_OPEN);
        }
    }

    private void setClaw(Gamepad gamepad) {
        if(gamepad.dpad_down){
            clawServo.setPosition(MAX_CLAW_CLOSE);
        }else  if(gamepad.dpad_up) {
            clawServo.setPosition(MAX_CLAW_OPEN);
        }
    }

}
