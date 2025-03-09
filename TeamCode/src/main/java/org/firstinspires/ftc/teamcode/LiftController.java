package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftController {
    private DcMotor liftMotor;

    // Ограничители (максимальные значения)
    private int minPosition = 22; // Минимальная высота
    private int maxPosition = -11500; // Максимальная высота

    // Скорость мотора
    private final double LIFT_SPEED = 1.0;

    // Инициализация мотора лифта
    public void init(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Управление лифтом с триггерами L2 и R2
    public void controlLift(double leftTrigger, double rightTrigger) {
        double power = rightTrigger - leftTrigger; // R2 поднимает, L2 опускает
        liftMotor.setPower(power * LIFT_SPEED); // Управление скоростью
    }

    // Возвращает текущую позицию лифта
    public int getLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void setLimits(int minValue, int maxValue) {
    }
}