package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpinMotor {
    private DcMotor spinMotor;

    // Инициализация мотора спиннера
    public void init(HardwareMap hardwareMap) {
        spinMotor = hardwareMap.get(DcMotor.class, "spinMotor"); // Имя мотора в конфиге
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Управление спиннером с триггеров второго геймпада
    public void controlSpin(double power) {
        spinMotor.setPower(power);
    }
}
