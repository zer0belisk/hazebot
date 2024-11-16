package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "RobotTeleOp", group = "TeleOp")
public class RobotTeleOp extends OpMode {
    private RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Чтение значений с геймпадаf
        double drive = -gamepad1.left_stick_y; // Вперед/назад
        double strafe = gamepad1.left_stick_x; // Лево/право
        double turn = -gamepad1.right_stick_x; // Поворот

        // Расчет мощности для каждого мотора
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftRearPower = drive - strafe + turn;
        double rightRearPower = drive + strafe - turn;

        // Нормализация мощности, чтобы избежать превышения диапазона [-1, 1]
        double maxPower = Math.max(1.0, Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)))));

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightRearPower /= maxPower;

        // Установка мощности моторам
        robot.leftFrontMotor.setPower(leftFrontPower);
        robot.rightFrontMotor.setPower(rightFrontPower);
        robot.leftRearMotor.setPower(leftRearPower);
        robot.rightRearMotor.setPower(rightRearPower);

        // Отображение данных на экране
        telemetry.addData("Motors", "LF: %.2f, RF: %.2f, LR: %.2f, RR: %.2f",
                leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        telemetry.update();
    }
}
