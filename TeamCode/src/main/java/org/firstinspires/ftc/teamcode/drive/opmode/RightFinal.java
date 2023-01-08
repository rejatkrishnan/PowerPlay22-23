package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@Autonomous

public class RightFinal extends LinearOpMode {
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
    private double CrUpperUpdate = 163;
    private double CbUpperUpdate = 172;
    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip
    private double lowerruntime = 0;
    private double upperruntime = 0;

    //cone color
    public static Scalar scalarLowerYCrCbCone = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCbCone = new Scalar(255.0, 170.0, 120.0);
    //pole color
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 152, 46);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 163, 172);

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 2.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.2,65.6, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36, 0))
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-36.1, 15.4))
                .lineToLinearHeading(new Pose2d(-32.6, 12.6, Math.toRadians(-33.5)))
                .addTemporalMarker(0.5, () -> {
                    setAngle(0.8f, 500);
                })
                .addTemporalMarker(1, () -> {
                    Lift(1,3000);
                })
                .addTemporalMarker(2, this::findPole)
                .waitSeconds(4)
                .addTemporalMarker(3.1, () -> {
                    Lift(0.4f, 440);
                    grip(-0.5f);
                })

                .build();


        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.setMsTransmissionInterval(50);

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
        telemetry.addData("Starting at", "%7d :%7d");
        leftFront.getCurrentPosition();
        rightFront.getCurrentPosition();
        rightRear.getCurrentPosition();
        leftRear.getCurrentPosition();
        lift = hardwareMap.dcMotor.get("l");
        angle = hardwareMap.dcMotor.get("a");
        grip1 = hardwareMap.get(CRServo.class, "g1");
        grip2 = hardwareMap.get(CRServo.class, "g2");
        //RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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


        float driveSpeed = 0.75f; //sets drive motor speeds (between 0 and 1)
        double armSpeedUp = 1;
        double armSpeedDown = -0.4;

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();


            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        waitForStart();

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2);
        } else {
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2);
        }
        while (opModeIsActive()) {
        }
    }
    public void setAngle(float power, int timeLimit) {
        angle.setPower(power);
        sleep(timeLimit);
        angle.setPower(0);
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
        robotPower(-0.2f, 0.2f);
        telemetry.addLine("Autonomous A");
    }
    public void AUTONOMOUS_B() {
        robotPower(0, 0);
        telemetry.addLine("Autonomous B");
    }
    public void AUTONOMOUS_C() {
        robotPower(0.2f, -0.2f);
        telemetry.addLine("Autonomous C");
    }
    public void robotPower(float leftPower, float rightPower) {
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
    }
    private void AUTOFORWARD() {
        robotPower(0.25f, 0.25f);
    }
    private void AUTOBACKWARD() {
        robotPower(-0.25f, -0.25f);
    }
    public void Lift(float power, int target) {
        Telemetry.Item liftPosition = telemetry.addData("Lift Position", lift.getCurrentPosition());
        lift.setPower(power);
        lift.setTargetPosition(target);
        liftPosition.setValue(lift.getCurrentPosition());
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.update();
    }
    private void grip(float power) {
        grip1.setPower(power);
        grip2.setPower(-power);
    }
    public void findPole() {
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if (myPipeline.error) {
            telemetry.addData("Exception: ", myPipeline.debug);
        }
        telemetry.addData("RectArea: ", myPipeline.getRectArea());
        telemetry.update();
        while(myPipeline.getRectArea() > 0)
            if (myPipeline.getRectArea() > 2000) {
            if (myPipeline.getRectMidpointX() > 100) {
                AUTONOMOUS_C();
            } else if (myPipeline.getRectMidpointX() > 0) {
                AUTONOMOUS_B();
               break;
            } else {
                AUTONOMOUS_A();
            }
        }

        while (myPipeline.getRectMidpointX() < 290 && myPipeline.getRectMidpointX() > 270) {
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
    public void findCone() {
            ContourPipeline myPipeline;
            webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
            // Configuration of Pipeline
            myPipeline.configureScalarLower(scalarLowerYCrCbCone.val[0], scalarLowerYCrCbCone.val[1], scalarLowerYCrCbCone.val[2]);
            myPipeline.configureScalarUpper(scalarUpperYCrCbCone.val[0], scalarUpperYCrCbCone.val[1], scalarUpperYCrCbCone.val[2]);
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();
            //needs to be configured tomorrow
            while(myPipeline.getRectArea() > 0)
                if (myPipeline.getRectArea() > 2000) {
                    if (myPipeline.getRectMidpointX() > 290) {
                        AUTONOMOUS_C();
                    } else if (myPipeline.getRectMidpointX() > 270) {
                        AUTONOMOUS_B();
                        break;
                    } else {
                        AUTONOMOUS_A();
                    }
                }

            while (myPipeline.getRectMidpointX() < 285 && myPipeline.getRectMidpointX() > 260) {
                if (myPipeline.getRectArea() > 15200) {
                    AUTOBACKWARD();
                } else if (myPipeline.getRectArea() < 14100) {
                    AUTOFORWARD();
                } else {
                    AUTOSCORE();
                    break;
                    //end configuration of cone
                }
            }
        }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
