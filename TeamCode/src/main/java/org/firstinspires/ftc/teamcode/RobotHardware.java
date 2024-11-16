package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;

    public void init(HardwareMap hardwareMap) {
        // Инициализация моторов
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftrear_motor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightrear_motor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftfront_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront_motor");

        // Установка направления моторов
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        // Установка режима работы моторов
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Установка начальной мощности
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }
}
