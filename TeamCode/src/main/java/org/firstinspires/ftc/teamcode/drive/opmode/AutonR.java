package org.firstinspires.ftc.teamcode.drive.opmode;

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

@Autonomous

public class AutonR extends LinearOpMode {

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


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    int LIFT_POS_GRAB = 0;
    int LIFT_POS_HIGH = 3000;
    int LIFT_POS_MEDIUM = 2150;
    int LIFT_POS_LOW = 1300;
    int step = 100;


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
                .lineToLinearHeading(
                        new Pose2d(-31.8, 11.3, Math.toRadians(-38.5)),
                SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.5, () -> {
                    setAngle(0.8f, 500);
                })
                .addTemporalMarker(1, () -> {
                    Lift(1,3000);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(4, () -> {
                    Lift(0.4f, 440);
                    grip(-0.5f);
                })

                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-33, 24, Math.toRadians(-37)))
                .splineToLinearHeading(new Pose2d(-46, 20, Math.toRadians(240)), Math.toRadians(180))
                .addDisplacementMarker(4, () -> {
                    grip(0.5f);
                })
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-52, 20, Math.toRadians(240)))
                .addTemporalMarker(0.45, () -> {
                    Lift(1, 3000);
                    grip(0);
                })
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(-52, 20, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-31.8, 11.3, Math.toRadians(-38.5)))
                .addTemporalMarker(3.3, () -> {
                    Lift(0.4f, 365);
                    grip(-0.5f);
                })
                .waitSeconds(0.05)
                .addTemporalMarker(3.65, () -> {
                    grip(-1);
                })
                .waitSeconds(0.4)
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-33, 24, Math.toRadians(-37)))
                .splineToLinearHeading(new Pose2d(-43, 13, Math.toRadians(260)), Math.toRadians(180))
                .addDisplacementMarker(1, () -> {
                    grip(0.5f);
                })
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .lineToLinearHeading(new Pose2d(-53, 13, Math.toRadians(260)))
                .addTemporalMarker(0.45, () -> {
                    Lift(1, 3000);
                    grip(0.01f);
                })
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(-35.5)))
                .addTemporalMarker(3.7, () -> {
                    Lift(0.4f, 330);
                    grip(-0.5f);
                })
                .waitSeconds(0.3)
                .build();
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-33, 24, Math.toRadians(-37)))
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(200.2)), Math.toRadians(180))
                .addDisplacementMarker(1, () -> {
                    grip(0.5f);
                })
                .build();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .lineToLinearHeading(new Pose2d(-53.5, 13, Math.toRadians(200.2)))
                .addTemporalMarker(0.45, () -> {
                    Lift(1, 3000);
                    grip(0.01f);
                })
                .build();
        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj10.end())
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(-35.5)))
                .addTemporalMarker(3.65, () -> {
                    Lift(0.4f, 0);
                    grip(-0.5f);
                })
                .waitSeconds(0.3)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_LEFT);
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



        /* Actually do something useful */
        waitForStart();



        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2);
            drive.followTrajectorySequence(traj3);
            drive.followTrajectorySequence(traj4);
            drive.followTrajectorySequence(traj5);
            drive.followTrajectorySequence(traj6);
            drive.followTrajectorySequence(traj7);
            drive.followTrajectorySequence(traj8);
            drive.followTrajectorySequence(traj9);
            drive.followTrajectorySequence(traj10);
            drive.followTrajectorySequence(traj11);

        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2);
            drive.followTrajectorySequence(traj3);
            drive.followTrajectorySequence(traj4);
            drive.followTrajectorySequence(traj5);
            drive.followTrajectorySequence(traj6);
            drive.followTrajectorySequence(traj7);
            drive.followTrajectorySequence(traj8);
            drive.followTrajectorySequence(traj9);
            drive.followTrajectorySequence(traj10);
            drive.followTrajectorySequence(traj11);

        } else {
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2);
            drive.followTrajectorySequence(traj3);
            drive.followTrajectorySequence(traj4);
            drive.followTrajectorySequence(traj5);
            drive.followTrajectorySequence(traj6);
            drive.followTrajectorySequence(traj7);
            drive.followTrajectorySequence(traj8);
            drive.followTrajectorySequence(traj9);
            drive.followTrajectorySequence(traj10);
            drive.followTrajectorySequence(traj11);
        }

        while (opModeIsActive()) {
            sleep(20);
        }
    }





    public void setAngle(float power, int timeLimit) {
        angle.setPower(power);
        sleep(timeLimit);
        angle.setPower(0);
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



    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        //telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        //telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        //telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        //telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        //telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        //telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        //telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
