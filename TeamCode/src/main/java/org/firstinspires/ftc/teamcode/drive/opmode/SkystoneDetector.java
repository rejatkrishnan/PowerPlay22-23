package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
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