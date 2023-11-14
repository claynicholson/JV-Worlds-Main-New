package org.firstinspires.ftc.teamcode.drive.runnable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {
    Pose2d poseEstimate = new Pose2d(0, 0, Math.toRadians(0));
    protected DcMotor leftLift;
    protected Servo leftClaw, rightClaw;
    public static double destX = 53.06;
    public static double destY = -0.3947;
    public static double destH = 0.8159;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(poseEstimate);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(-0.25);

        waitForStart();
        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                .addTemporalMarker(0, () -> {
                    rightClaw.setPosition(0);
                    leftClaw.setPosition(0.5);
                })
                .lineToLinearHeading(new Pose2d(50.574049349703046, -4.281011379928819 , Math.toRadians(48.61271016484739)))
                .addTemporalMarker(1.5, () -> {
                    leftLift.setTargetPosition(915);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.75);
                })
                .build();
        drive.followTrajectory(traj1);
        poseEstimate = drive.getPoseEstimate();


        Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(destX, destY , destH), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        drive.followTrajectory(traj2);
        poseEstimate = drive.getPoseEstimate();

        TrajectorySequence traj205 = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.8)
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(500);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftClaw.setPosition(0.25);
                    rightClaw.setPosition(0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftLift.setTargetPosition(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })
                .lineToLinearHeading(new Pose2d(47.7, -3.7 , 4.67426 ))
                .build();
        drive.followTrajectorySequence(traj205);
        poseEstimate = drive.getPoseEstimate();

        //2nd cone
        Trajectory traj3 = drive.trajectoryBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(51.9221, -25.77 , 4.67426))
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftLift.setTargetPosition(130);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                })
                .build();


        drive.followTrajectory(traj3);
        telemetry.addData("armPos" , leftLift.getCurrentPosition());
        telemetry.update();
        poseEstimate = drive.getPoseEstimate();

        TrajectorySequence traj4  = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.2)
                .addTemporalMarker(0, () -> {
                    leftClaw.setPosition(0.5);
                    rightClaw.setPosition(0);

                    telemetry.addData("armPos" , leftLift.getCurrentPosition());
                    telemetry.update();
                })
                .lineToLinearHeading(new Pose2d(51.9221, -24.77 , 4.67426))

                .addTemporalMarker(0.5, () -> {
                    leftLift.setTargetPosition(400);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.5);
                })
                .build();
        drive.followTrajectorySequence(traj4);
        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(poseEstimate)

                .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                .build();
        drive.followTrajectorySequence(traj5);
        poseEstimate = drive.getPoseEstimate();
        Trajectory traj6 = drive.trajectoryBuilder(poseEstimate)
                .addTemporalMarker(0.6 , () -> {
                    leftLift.setTargetPosition(915);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(destX, destY , destH ))
                .build();
        drive.followTrajectory(traj6);

        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.8)
                .lineToLinearHeading(new Pose2d(47.7, -3.7 , 4.67426 ))
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(500);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftClaw.setPosition(0.25);
                    rightClaw.setPosition(0.25);
                })


                .build();
        drive.followTrajectorySequence(traj7);


        poseEstimate = drive.getPoseEstimate();
        //3rd cone
        Trajectory traj8 = drive.trajectoryBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(51.9221, -24.77 , 4.67426))
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftLift.setTargetPosition(115);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                })
                .build();


        drive.followTrajectory(traj8);
        telemetry.addData("armPos" , leftLift.getCurrentPosition());
        telemetry.update();
        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.35)
                .addTemporalMarker(0, () -> {
                    leftClaw.setPosition(0.5);
                    rightClaw.setPosition(0);
                })
                .addTemporalMarker(0.3, () -> {
                    leftLift.setTargetPosition(400);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.5);
                })
                .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                .build();
        drive.followTrajectorySequence(traj10);
        poseEstimate = drive.getPoseEstimate();
        Trajectory traj11 = drive.trajectoryBuilder(poseEstimate)
                .addTemporalMarker(0.6 , () -> {
                    leftLift.setTargetPosition(915);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(destX, destY , destH ))
                .build();
        drive.followTrajectory(traj11);

        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.8)
                .lineToLinearHeading(new Pose2d(50.574049349703046, -4.281011379928819 , Math.toRadians(48.61271016484739)))
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(500);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftClaw.setPosition(0.25);
                    rightClaw.setPosition(0.25);
                })


                .build();
        drive.followTrajectorySequence(traj12);
        poseEstimate = drive.getPoseEstimate();
        //4th cone
        Trajectory traj13 = drive.trajectoryBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(51.9221, -23 , 4.67426))
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.25);
                })

                .addTemporalMarker(0.5, () -> {
                    leftLift.setTargetPosition(100);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(1);
                })
                .build();


        drive.followTrajectory(traj13);
        telemetry.addData("armPos" , leftLift.getCurrentPosition());
        telemetry.update();
        poseEstimate = drive.getPoseEstimate();
        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.35)
                .addTemporalMarker(0, () -> {
                    leftClaw.setPosition(0.5);
                    rightClaw.setPosition(0);

                    telemetry.addData("armPos" , leftLift.getCurrentPosition());
                    telemetry.update();
                })
                .addTemporalMarker(0.3, () -> {
                    leftLift.setTargetPosition(400);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.5);
                })
                .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                .build();
        drive.followTrajectorySequence(traj15);
        poseEstimate = drive.getPoseEstimate();
        Trajectory traj16 = drive.trajectoryBuilder(poseEstimate)
                .addTemporalMarker(0.6 , () -> {
                    leftLift.setTargetPosition(915);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(destX, destY , destH ))
                .build();
        drive.followTrajectory(traj16);

        poseEstimate = drive.getPoseEstimate();
        TrajectorySequence traj17 = drive.trajectorySequenceBuilder(poseEstimate)
                .waitSeconds(0.8)
                .lineToLinearHeading(new Pose2d(50.574049349703046, -4.281011379928819 , Math.toRadians(48.61271016484739)))
                .addTemporalMarker(0, () -> {
                    leftLift.setTargetPosition(500);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })
                .addTemporalMarker(0.5, () -> {
                    leftClaw.setPosition(0.25);
                    rightClaw.setPosition(0.25);
                })

                .addTemporalMarker(1, () -> {
                    leftLift.setTargetPosition(0);
                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLift.setPower(-0.25);
                })


                .build();
        drive.followTrajectorySequence(traj17);
        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()){
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("armPos" , leftLift.getCurrentPosition());
            telemetry.update();
        }
    }
}