package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import fi.iki.elonen.NanoHTTPD;

@TeleOp(name = "NanoHttpServer", group = "TeleOp")
public class NanoHttpServer extends OpMode {

    private NanoHTTPD server;

    @Override
    public void init() {
        try {
            // Инициализируем сервер на порту 8080
            server = new NanoHTTPD(8451) {
                @Override
                public Response serve(IHTTPSession session) {
                    // Обработка запроса и ответ
                    String response = "Hello from FTC Robot!";
                    return newFixedLengthResponse(Response.Status.OK, "text/plain", response);
                }
            };
            // Запускаем сервер
            server.start();
            telemetry.addData("Server Status", "Started on port 8080");
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
        // Выводим статус сервера
        telemetry.addData("Status", "Server running");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Останавливаем сервер при завершении программы
        if (server != null) {
            server.stop();
            telemetry.addData("Server Status", "Stopped");
        }
    }
}
