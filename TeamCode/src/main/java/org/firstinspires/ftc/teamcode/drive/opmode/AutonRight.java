package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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



@Config
@Autonomous(group = "drive")
public class AutonRight extends LinearOpMode {

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


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(50)
                .addDisplacementMarker(30, () -> {
                    Lift(1, 3000);
                })
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(35))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                        .forward(4.75)
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            Lift(0.4f, 500);
                            grip(-0.5f);
        })
                .waitSeconds(1.5)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .waitSeconds(0.25)
                .back(6)
                .addDisplacementMarker(() -> {
                    grip(0);
                })
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .turn(Math.toRadians(-35))
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
                .strafeRight(15)
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(traj6.end())
                .turn(Math.toRadians(-93))
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())
                .strafeLeft(3)
                .addDisplacementMarker(() -> {
                    grip(0.4f);
                })
                .build();
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(traj8.end())
                .forward(8)
                .build();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(traj9.end())
                .addDisplacementMarker(() -> {
                    grip(0);
                    Lift(1, 3000);

                })
                .back(10)
                .build();
        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj10.end())
                .turn(Math.toRadians(160))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(traj11.end())
                .forward(12.5)
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(traj12.end())
                .addDisplacementMarker(() -> {
                    Lift(0.4f, 320);
                    grip(-0.45f);
                })
                .back(11.5)
                .build();
        TrajectorySequence traj14 = drive.trajectorySequenceBuilder(traj13.end())
                .turn(Math.toRadians(-170))
                .addDisplacementMarker(() -> {
                    grip(0.45f);
                })
                .build();
        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj14.end())
                .forward(9)
                .build();
        TrajectorySequence traj16 = drive.trajectorySequenceBuilder(traj15.end())
                .addDisplacementMarker(() -> {
                    grip(0);
                    Lift(1, 3000);
                })
                .back(10)
                .build();
        TrajectorySequence traj17 = drive.trajectorySequenceBuilder(traj16.end())
                .turn(Math.toRadians(170))
                .build();
        TrajectorySequence traj18 = drive.trajectorySequenceBuilder(traj17.end())
                .forward(9)
                .build();
        TrajectorySequence traj19 = drive.trajectorySequenceBuilder(traj18.end())
                .addDisplacementMarker(() -> {
                    Lift(0.4f, 250);
                    grip(-0.45f);
                })
                .back(11.5)
                .build();

        TrajectorySequence traj20 = drive.trajectorySequenceBuilder(traj19.end())
                .turn(Math.toRadians(-170))
                .addDisplacementMarker(() -> {
                    grip(0.45f);
                })
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj20.end())
                .forward(9)
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj21.end())
                .addDisplacementMarker(() -> {
                    grip(0);
                    Lift(1, 3000);
                })
                .back(10)
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj22.end())
                .turn(Math.toRadians(170))
                .build();
        TrajectorySequence traj24 = drive.trajectorySequenceBuilder(traj23.end())
                .forward(9)
                .build();
        TrajectorySequence traj25 = drive.trajectorySequenceBuilder(traj24.end())
                .addDisplacementMarker(() -> {
                    Lift(0.4f, 130);
                    grip(-0.45f);
                })
                .back(11.5)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
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
        drive.followTrajectorySequence(traj12);
        drive.followTrajectorySequence(traj13);
        drive.followTrajectorySequence(traj14);
        drive.followTrajectorySequence(traj15);
        drive.followTrajectorySequence(traj16);
        drive.followTrajectorySequence(traj17);
        drive.followTrajectorySequence(traj18);
        drive.followTrajectorySequence(traj19);
        drive.followTrajectorySequence(traj20);
        drive.followTrajectorySequence(traj21);
        drive.followTrajectorySequence(traj22);
        drive.followTrajectorySequence(traj23);
        drive.followTrajectorySequence(traj24);
        drive.followTrajectorySequence(traj25);



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
