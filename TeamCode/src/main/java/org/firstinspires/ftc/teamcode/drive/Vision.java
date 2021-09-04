package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Size;

import java.lang.reflect.Array;
import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Vision extends LinearOpMode
{
    OpenCvCamera webcam;
    TOP_TOWER_COLOR top_tower_color;

    public enum TOP_TOWER_COLOR {
        BLUE,
        RED,
        YELLOW
    }

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        Pipeline pipeline = new Pipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("COLOR", pipeline.getTallestColor().toString());
            telemetry.addData("COLOR", pipeline.getTallestColor().toString());

            telemetry.update();

            sleep(100);
        }
    }

    class Pipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        Mat mat = new Mat();

        Scalar lowBlueHSV = new Scalar(100,150,0);
        Scalar highBlueHSV = new Scalar(140,255,255);

        Scalar lowYellowHSV = new Scalar(20,100,100);
        Scalar highYellowHSV = new Scalar(30,255,255);

        Scalar lowRedHSV = new Scalar(155,25,0);
        Scalar highRedHSV = new Scalar(179,255,255);

        private int findColorPosition(Scalar lowerColor, Scalar higherColor, Mat input) {
            int Y = 0;

            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            // variable to store mask in
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
            Core.inRange(mat, lowerColor, higherColor, mask);

            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            int maxWidth = 0;
            Rect maxRect = new Rect();
            for (MatOfPoint c : contours) {
                MatOfPoint copy = new MatOfPoint(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                int w = rect.width;
                // checking if the rectangle is below the horizon
                if (w > maxWidth) {
                    maxWidth = w;
                    maxRect = rect;
                }

                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            Y = maxRect.y;

            return Y;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            int bluePos = findColorPosition(lowBlueHSV, highBlueHSV, input);
            int yellowPos = findColorPosition(lowYellowHSV, highYellowHSV, input);
            int redPos = findColorPosition(lowRedHSV, highRedHSV, input);

            if(bluePos > yellowPos && bluePos > redPos) {
                top_tower_color = TOP_TOWER_COLOR.BLUE;
            }

            if(yellowPos > bluePos && yellowPos > redPos) {
                top_tower_color = TOP_TOWER_COLOR.YELLOW;
            }

            if(redPos > yellowPos && redPos > bluePos) {
                top_tower_color = TOP_TOWER_COLOR.RED;
            }

            return input;
        }

        public TOP_TOWER_COLOR getTallestColor()
        {
            return top_tower_color;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}