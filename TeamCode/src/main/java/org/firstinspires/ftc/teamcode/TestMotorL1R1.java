package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestMotorL1R1", group = "Test")
public class TestMotorL1R1 extends OpMode {

    private DcMotor motorL1R1;

    @Override
    public void init() {
        // Инициализация мотора
        motorL1R1 = hardwareMap.get(DcMotor.class, "motorL1R1");
        motorL1R1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL1R1.setPower(0);
        motorL1R1.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Управление мотором L1/R1
        if (gamepad1.left_bumper) {
            motorL1R1.setPower(-1); // Опускание
        } else if (gamepad1.right_bumper) {
            motorL1R1.setPower(1); // Подъем
        } else {
            motorL1R1.setPower(0); // Остановка
        }

        // Отображение телеметрии
        telemetry.addData("Motor Power", motorL1R1.getPower());
        telemetry.update();
    }
}
