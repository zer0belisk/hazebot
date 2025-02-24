package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoController {
    private Servo spinnerServo;
    private Servo plateServo;

    private boolean spinnerState = false; // Текущее состояние спиннера
    private boolean plateState = false;   // Текущее состояние пластины

    private boolean lastSpinnerButton = false; // Для отслеживания нажатия кнопки
    private boolean lastPlateButton = false;   // Для отслеживания нажатия кнопки

    // Инициализация сервоприводов
    public void init(HardwareMap hardwareMap) {
        spinnerServo = hardwareMap.get(Servo.class, "spinner");
        plateServo = hardwareMap.get(Servo.class, "plate");

        // Устанавливаем начальные позиции
        spinnerServo.setPosition(0.0);
        plateServo.setPosition(0.0);
    }

    // Управление "spinner" (вращение) одной кнопкой
    public void controlSpinner(boolean buttonPressed) {
        if (buttonPressed && !lastSpinnerButton) { // Проверка нажатия кнопки
            spinnerState = !spinnerState; // Переключение состояния
            spinnerServo.setPosition(spinnerState ? 1.0 : 0.0);
        }
        lastSpinnerButton = buttonPressed; // Запоминаем состояние кнопки
    }

    // Управление "plate" (наклон) одной кнопкой
    public void controlPlate(boolean buttonPressed) {
        if (buttonPressed && !lastPlateButton) { // Проверка нажатия кнопки
            plateState = !plateState; // Переключение состояния
            plateServo.setPosition(plateState ? 1.0 : 0.0);
        }
        lastPlateButton = buttonPressed; // Запоминаем состояние кнопки
    }
}
