package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.MecanumWheelsAuton;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * Second chassis only has 4 dc motors. No arm motor or any other servo.
 */
@TeleOp(name = "TeleopBaseChassis", group = "FuriousFrogs24")
public class OpModeBaseChassisTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;


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

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double chassisY = getChassisY();
            double chassisX = getChassisX();
            double chassisTurn = gamepad1.right_stick_x;

            armMotor.setPower(gamepad2.left_stick_x);
            armMotor2.setPower(gamepad2.left_stick_y);
            clawServo.setPosition(gamepad2.left_trigger);
            wristServo.setPosition(gamepad2.right_trigger);

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
