package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "hazecontrol" , group = "TeleOp")
public class DriveControl extends OpMode {

    private RobotHardware robot = new RobotHardware();
    private ArmController armController = new ArmController();
    private LiftController liftController = new LiftController(); // Добавляем управление лифтом
    private ServoController servoController = new ServoController();
    private SpinMotor spinMotor = new SpinMotor();

    @Override
    public void init() {
        robot.init(hardwareMap);
        armController.init(hardwareMap);
        liftController.init(hardwareMap); // Инициализируем лифт
        servoController.init(hardwareMap);
        spinMotor.init(hardwareMap);

        liftController.setLimits(Integer.MIN_VALUE, Integer.MAX_VALUE); // Макс. значения для теста

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // --- Управление движением робота ---
        double driveY = gamepad1.left_stick_y; // Вперед/назад
        double driveX = -gamepad1.left_stick_x; // Вправо/влево
        double turn = -gamepad1.right_stick_x; // Поворот

        // Управление сервоприводами
        servoController.controlSpinner(gamepad2.circle);
        servoController.controlPlate(gamepad2.square);

        // Расчет мощности для моторов
        double leftFrontPower = driveY + driveX + turn;
        double rightFrontPower = driveY - driveX - turn;
        double leftRearPower = driveY - driveX + turn;
        double rightRearPower = driveY + driveX - turn;

        // Нормализация мощности
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

        // --- Управление рукой ---
        if (gamepad2.right_bumper) {
            armController.controlArm(1.0); // Подъем руки
        } else if (gamepad2.left_bumper) {
            armController.controlArm(-1.0); // Опускание руки
        } else {
            armController.controlArm(0.0); // Остановка руки
        }

        // --- Управление спиннером (геймпад 2, R2/L2) ---
        if (gamepad2.right_trigger > 0.1) {
            spinMotor.controlSpin(gamepad2.right_trigger); // Крутить по часовой стрелке (R2)
        } else if (gamepad2.left_trigger > 0.1) {
            spinMotor.controlSpin(-gamepad2.left_trigger); // Крутить против часовой стрелки (L2)
        } else {
            spinMotor.controlSpin(0); // Остановить спиннер
        }


        liftController.controlLift(gamepad1.right_trigger, gamepad1.left_trigger);

        // --- Вывод данных на Driver Station ---
        telemetry.addData("Motor Power", "LF: %.2f, RF: %.2f, LR: %.2f, RR: %.2f",
                leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        telemetry.addData("Joystick", "X: %.2f, Y: %.2f", driveX, driveY);
        telemetry.addData("Arm Position", armController.getArmPosition());
        telemetry.addData("Lift Position", liftController.getLiftPosition());


        telemetry.update();
    }
}