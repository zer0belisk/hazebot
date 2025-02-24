package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class '/'ArmController {
    private DcMotor armMotor;

    // Лимиты движения (можно менять вручную)
    private int minPosition = -33000; // Минимальная позиция
    private int maxPosition = 33000;  // Максимальная позиция

    // Инициализация мотора
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Установка новых лимитов (если нужно)
    public void setLimits(int min, int max) {
        this.minPosition = min;
        this.maxPosition = max;
    }

    // Управление мотором с учетом ограничений
    public void controlArm(double power) {
        int currentPosition = armMotor.getCurrentPosition();

        // Ограничение на движение вверх
        if (currentPosition >= maxPosition && power > 0) {
            armMotor.setPower(0);
        }
        // Ограничение на движение вниз
        else if (currentPosition <= minPosition && power < 0) {
            armMotor.setPower(0);
        } else {
            armMotor.setPower(power);
        }
    }

    // Получение текущей позиции мотора
    public int getArmPosition() {
        return armMotor.getCurrentPosition();
    }
}
