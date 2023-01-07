package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous
public class SkystoneAuto extends LinearOpMode{
    private OpenCvCamera Camera;
    public SkystoneDetector SkystoneDetector;
    private String position;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;


    public abstract class SkystoneDetector extends OpenCvPipeline {
        private Mat workingMatrix = new Mat();
        public String position = "Left";
        public SkystoneDetector() {

        }


        @Override
        public final Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) {
                return input;
            }

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

            Mat matLeft = workingMatrix.submat(120, 150, 10, 50);
            Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
            Mat matRight = workingMatrix.submat(120, 150, 150, 190);

            Imgproc.rectangle(workingMatrix, new Rect(10,120,40,30), new Scalar(0,255,0));
            Imgproc.rectangle(workingMatrix, new Rect(80,120,40,30), new Scalar(0,255,0));
            Imgproc.rectangle(workingMatrix, new Rect(150,120,40,30), new Scalar(0,255,0));

            double leftTotal = Core.sumElems(matLeft).val[2];
            double centerTotal = Core.sumElems(matCenter).val[2];
            double rightTotal = Core.sumElems(matRight).val[2];

            if (leftTotal < centerTotal) {
                if (leftTotal < rightTotal) {
                    position = "Left";
                } else {
                    position = "Right";
                }
            }
            else {
                if(centerTotal < rightTotal) {
                    position = "Center";
                } else {
                    position = "Right";
                }
            }
            return workingMatrix;
        }
    }

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Camera.setPipeline(SkystoneDetector);
        Camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });

        while (!isStarted()) {
            position = position;
            telemetry.addData("position", position);
        }
        //code while robot is running
        if (position.equals ("Left")) {

        }

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);



    }
}