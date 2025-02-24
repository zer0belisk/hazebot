package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtraMotors {

    private DcMotor motorL1R1; // Мотор, управляемый L1 и R1
    private DcMotor motorL2R2; // Мотор, управляемый L2 и R2

    public void init(HardwareMap hardwareMap) {
        motorL1R1 = hardwareMap.get(DcMotor.class, "motorL1R1");
        motorL2R2 = hardwareMap.get(DcMotor.class, "motorL2R2");

        motorL1R1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorL2R2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorL1R1.setPower(0);
        motorL2R2.setPower(0);

        motorL1R1.setDirection(DcMotor.Direction.FORWARD);
        motorL2R2.setDirection(DcMotor.Direction.FORWARD);
    }

    // Управление мотором L1/R1
    public void controlMotorL1R1(boolean l1Pressed, boolean r1Pressed) {
        if (l1Pressed) {
            motorL1R1.setPower(-1); // Опускание
        } else if (r1Pressed) {
            motorL1R1.setPower(1); // Подъем
        } else {
            motorL1R1.setPower(0); // Остановка
        }
    }

    // Управление мотором L2/R2
    public void controlMotorL2R2(float l2Value, float r2Value) {
        double power = r2Value - l2Value; // Рассчитываем мощность
        motorL2R2.setPower(power); // Устанавливаем мощность
    }
}
