package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config //Disable if not using FTC Dashboard https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
@Autonomous(name="OpenCV_Contour_3954_Test", group="Tutorials")

public class OpenCVTest extends LinearOpMode {
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 152;
    private double CbLowerUpdate = 46;
    private double CrUpperUpdate = 1163;
    private double CbUpperUpdate = 172;

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Yellow Range
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "fl");
        rightFront = hardwareMap.get(DcMotor.class, "fr");
        leftRear = hardwareMap.get(DcMotor.class, "bl");
        rightRear = hardwareMap.get(DcMotor.class, "br");

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 290) {
                    AUTONOMOUS_C();
                    String camPosition = "";
                    telemetry.addData(camPosition, "");
                    while (camPosition != "B") {
                        camPosition = findPole(myPipeline);
                        telemetry.update();
                    }
                    if (myPipeline.getRectMidpointX() > 260) {
                        telemetry.addData(camPosition, "");
                        while (camPosition != "B") {
                            camPosition = findPole(myPipeline);
                            telemetry.update();
                            AUTONOMOUS_B();
                        }
                    } else{
                            AUTONOMOUS_A();
                            telemetry.addData(camPosition, "");
                            while (camPosition != "B") {
                                camPosition = findPole(myPipeline);
                                telemetry.update();
                            }
                        }
                    }
                }
            }
        }

        public void testing (ContourPipeline myPipeline){
            if (lowerruntime + 0.05 < getRuntime()) {
                CrLowerUpdate += -gamepad1.left_stick_y;
                CbLowerUpdate += gamepad1.left_stick_x;
                lowerruntime = getRuntime();
            }
            if (upperruntime + 0.05 < getRuntime()) {
                CrUpperUpdate += -gamepad1.right_stick_y;
                CbUpperUpdate += gamepad1.right_stick_x;
                upperruntime = getRuntime();
            }

            CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
            CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
            CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
            CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

            myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
            myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

            telemetry.addData("lowerCr ", (int) CrLowerUpdate);
            telemetry.addData("lowerCb ", (int) CbLowerUpdate);
            telemetry.addData("UpperCr ", (int) CrUpperUpdate);
            telemetry.addData("UpperCb ", (int) CbUpperUpdate);
        }

        public Double inValues ( double value, double min, double max){
            if (value < min) {
                value = min;
            }
            if (value > max) {
                value = max;
            }
            return value;
        }

        public void AUTONOMOUS_A () {
            frontRight.setPower(0.2);
            backRight.setPower(0.2);
            frontLeft.setPower(-0.2);
            backLeft.setPower(-0.2);
            telemetry.addLine("Autonomous A");
        }

        public void AUTONOMOUS_B () {
            frontRight.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            telemetry.addLine("Autonomous B");
        }

        public void AUTONOMOUS_C () {
            frontRight.setPower(-0.2);
            backRight.setPower(-0.2);
            frontLeft.setPower(0.2);
            backLeft.setPower(0.2);
            telemetry.addLine("Autonomous C");
        }
        public String findPole (ContourPipeline myPipeline){
            String retvalue = "";
            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 320) {
                    AUTONOMOUS_C();
                    retvalue = "C";
                } else if (myPipeline.getRectMidpointX() > 280) {
                    AUTONOMOUS_B();
                    retvalue = "B";
                } else {
                    AUTONOMOUS_A();
                    retvalue = "A";
                }
            }
            return retvalue;
        }
    }

