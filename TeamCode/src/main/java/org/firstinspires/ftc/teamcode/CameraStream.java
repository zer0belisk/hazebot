package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

@TeleOp(name = "Camera Stream", group = "TeleOp")
public class CameraStream extends OpMode {
    private OpenCvWebcam webcam;

    @Override
    public void init() {
        // Получаем ID экрана для видеопотока
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        // Создаем объект камеры
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
        );

        // Устанавливаем обработчик (pipeline)
        webcam.setPipeline(new SamplePipeline());

        // Открываем камеру асинхронно
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void loop() {
        // Показываем статистику
        telemetry.addData("FPS", webcam.getFps());
        telemetry.addData("Total Frame Count", webcam.getFrameCount());
        telemetry.addData("Pipeline Time (ms)", webcam.getPipelineTimeMs());
        telemetry.update();
    }

    class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Если обработка не требуется, возвращаем кадр как есть
            return input;
        }
    }
}
