package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config //Disable if not using FTC Dashboard https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
@Autonomous



public class PoleDetectFinal extends LinearOpMode {
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    DcMotor lift = null;
    DcMotor angle = null;
    CRServo grip1 = null;
    CRServo grip2 = null;
    DigitalChannel digitalTouch = null;
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
        lift = hardwareMap.dcMotor.get("l");
        angle = hardwareMap.dcMotor.get("a");
        grip1 = hardwareMap.get(CRServo.class, "g1");
        grip2 = hardwareMap.get(CRServo.class, "g2");
        //RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setAutoClear(true);
        Telemetry.Item liftPosition = telemetry.addData("Lift Position", lift.getCurrentPosition());
        liftPosition.setValue(lift.getCurrentPosition());
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.update();




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose1 = new Pose2d(37.2,-67.6, Math.toRadians(90));


        drive.setPoseEstimate(startPose1);


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose1)
                .lineToLinearHeading(new Pose2d(37.2,-45, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(38,0, Math.toRadians(-180)))
                .build();



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

        drive.followTrajectorySequence(traj1);

        while (opModeIsActive()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();
            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 290) {
                    AUTONOMOUS_C();
                } else if (myPipeline.getRectMidpointX() > 270) {
                    AUTONOMOUS_B();
                    break;
                } else {
                    AUTONOMOUS_A();
                }

                while (myPipeline.getRectMidpointX() < 285 && myPipeline.getRectMidpointX() > 260){
                    if (myPipeline.getRectArea() > 15200) {
                        AUTOBACKWARD();
                    } else if (myPipeline.getRectArea() < 14100) {
                        AUTOFORWARD();
                    } else {
                        AUTOSCORE();
                        break;
                    }

                }
            }
        }

    }

    private void AUTOSCORE() {
        robotPower(0, 0);


    }
    public Double inValues(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }
    public void AUTONOMOUS_A() {
        robotPower(-0.13f,0.13f);
        telemetry.addLine("Autonomous A");
    }

    public void AUTONOMOUS_B() {
        robotPower(0,0);
        telemetry.addLine("Autonomous B");
    }

    public void AUTONOMOUS_C() {
        robotPower(0.13f, -0.13f);
        telemetry.addLine("Autonomous C");
    }
    public void robotPower(float leftPower, float rightPower){
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
    }
    private void AUTOFORWARD() {
        robotPower(0.13f, 0.13f);
    }

    private void AUTOBACKWARD() {
        robotPower(-0.13f, -0.13f);
    }
}