package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "DiagonalDriveControl", group = "TeleOp")
public class DriveControl extends OpMode {

    private RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Получение данных с левого стика (диагональное движение)
        double driveY = gamepad1.left_stick_y; // Вперед/назад
        double driveX = -gamepad1.left_stick_x;  // Вправо/влево
        double turn = -gamepad1.right_stick_x;   // Поворот

        // Управление движением
        double leftFrontPower = driveY + driveX + turn; // Диагональ вверх-вправо
        double rightFrontPower = driveY - driveX - turn; // Диагональ вверх-влево
        double leftRearPower = driveY - driveX + turn; // Диагональ вниз-вправо
        double rightRearPower = driveY + driveX - turn; // Диагональ вниз-влево

        // Нормализация мощности (если значение превышает 1)
        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightRearPower /= maxPower;

        // Установка мощности моторов
        robot.leftFrontMotor.setPower(leftFrontPower);
        robot.rightFrontMotor.setPower(rightFrontPower);
        robot.leftRearMotor.setPower(leftRearPower);
        robot.rightRearMotor.setPower(rightRearPower);

        // Отображение данных на Driver Station
        telemetry.addData("Motor Power", "LF: %.2f, RF: %.2f, LR: %.2f, RR: %.2f",
                leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        telemetry.addData("Joystick", "X: %.2f, Y: %.2f", driveX, driveY);
        telemetry.update();
    }
}
