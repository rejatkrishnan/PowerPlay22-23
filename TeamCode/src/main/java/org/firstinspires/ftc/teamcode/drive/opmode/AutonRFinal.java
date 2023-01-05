package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

/*
 * This is a simple routine to test translational drive capabilities.
 */


@Disabled
@Config
@Autonomous(group = "drive")
public class AutonRFinal extends LinearOpMode {

    DcMotor lift = null;
    CRServo grip1 = null;
    CRServo grip2 = null;
    DigitalChannel digitalTouch = null;

    int LIFT_POS_GRAB = 0;
    int LIFT_POS_HIGH = 3000;
    int LIFT_POS_MEDIUM = 2150;
    int LIFT_POS_LOW = 1300;
    int step = 100;
    boolean driving;



    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());



        lift = hardwareMap.dcMotor.get("l");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grip1 = hardwareMap.get(CRServo.class, "g1");
        grip2 = hardwareMap.get(CRServo.class, "g2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35.2,65.6, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

//start
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1, () -> {
                    Lift(1, 3000);
                })
                .lineToLinearHeading(new Pose2d(-31, 12.4, Math.toRadians(-37))) //drop 1st cone
                .addTemporalMarker(2.7, () -> {
                    Lift(0.4f, 440);
                    grip(-0.5f);
                })
                .waitSeconds(0.4)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-33, 24, Math.toRadians(-37)))
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(200.2)), Math.toRadians(180))
                .addDisplacementMarker(4, () -> {
                   grip(0.5f);
                })
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(-33, 23, Math.toRadians(-50)))
                .splineToLinearHeading(new Pose2d(-40, 15, Math.toRadians(330)), Math.toRadians(180))
                .addTemporalMarker(0.45, () -> {
                    Lift(1, 3000);
                    grip(0);
                })
                        .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-30.6, 11, Math.toRadians(-36.5)))
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
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-33, 24, Math.toRadians(-37)))
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(200.2)), Math.toRadians(180))
                .addDisplacementMarker(1, () -> {
                    grip(0.5f);
                })
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(-54, 13, Math.toRadians(200.2)))
                .addTemporalMarker(0.45, () -> {
                    Lift(1, 3000);
                    grip(0.01f);
                })
                        .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(-35.5)))
                .addTemporalMarker(3.7, () -> {
                    Lift(0.4f, 330);
                    grip(-0.5f);
                })
                .waitSeconds(0.3)
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(-33, 24, Math.toRadians(-37)))
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(200.2)), Math.toRadians(180))
                .addDisplacementMarker(1, () -> {
                    grip(0.5f);
                })
                .build();
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(-53.5, 13, Math.toRadians(200.2)))
                .addTemporalMarker(0.45, () -> {
                    Lift(1, 3000);
                    grip(0.01f);
                })
                .build();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .splineToLinearHeading(new Pose2d(-50, 13, Math.toRadians(0)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(-35.5)))
                .addTemporalMarker(3.65, () -> {
                    Lift(0.4f, 0);
                    grip(-0.5f);
                })
                .waitSeconds(0.3)
                .build();
        waitForStart();

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



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }



    private void grip(float power) {
        grip1.setPower(power);
        grip2.setPower(-power);

    }

    public void Lift(float power, int target) {
        Telemetry.Item liftPosition = telemetry.addData("Lift Position", lift.getCurrentPosition());
        lift.setPower(power);
        lift.setTargetPosition(target);
        liftPosition.setValue(lift.getCurrentPosition());
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.update();
    }

        }