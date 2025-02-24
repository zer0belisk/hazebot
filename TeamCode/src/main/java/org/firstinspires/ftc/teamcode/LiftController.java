package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftController {
    private DcMotor liftMotor;

    // Ограничители (максимальные значения для тестирования)
    private int minPosition = 22; // Минимальная высота (отрицательное значение, если нужно)
    private int maxPosition = -11500; // Максимальная высота (ограничение вверх)

    // Скорость мотора
    private final double LIFT_SPEED = 1.0;

    // Инициализация мотора лифта
    public void init(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Установка ограничений для лифта
    public void setLimits(int min, int max) {
        this.minPosition = -11500;
        this.maxPosition = 0;
    }

    // Управление мотором лифта с D-Pad (Up / Down)
    public void controlLift(boolean up, boolean down) {
        int currentPosition = liftMotor.getCurrentPosition();

        if (up && currentPosition < maxPosition) {
            liftMotor.setPower(LIFT_SPEED); // Движение вверх
        } else if (down && currentPosition > minPosition) {
            liftMotor.setPower(-LIFT_SPEED); // Движение вниз
        } else {
            liftMotor.setPower(0); // Остановка
        }
    }

    // Возвращает текущую позицию лифта
    public int getLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    // Возвращает минимальное ограничение
    public int getMinLimit() {
        return minPosition;
    }

    // Возвращает максимальное ограничение
    public int getMaxLimit() {
        return maxPosition;
    }
}
