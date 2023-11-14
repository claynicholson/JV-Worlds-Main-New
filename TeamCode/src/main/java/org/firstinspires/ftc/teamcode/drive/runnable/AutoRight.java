/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.runnable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@ Autonomous
public class AutoRight extends LinearOpMode
{
    Pose2d poseEstimate = new Pose2d(0, 0, Math.toRadians(0));
    protected DcMotor leftLift;
    protected Servo leftClaw, rightClaw;
    public static double destX = 53.59;
    public static double destY = -1.1699;
    public static double destH = 0.775;
    public static double Hchange = 0.2;

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
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(poseEstimate);
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(-0.25);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);



        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            //.lineToLinearHeading(new Pose2d(51.64, -3.585 , 4.711 ))

            Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0, () -> {
                        leftClaw.setPosition(0.5);
                        rightClaw.setPosition(0);
                    })
                    .lineToLinearHeading(new Pose2d(50.574049349703046, -4.281011379928819 , Math.toRadians(48.61271016484739)))
                    .addTemporalMarker(1.5, () -> {
                        leftLift.setTargetPosition(888);
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
                    .lineToLinearHeading(new Pose2d(49, -25 , 4.693))
                    .addTemporalMarker(0, () -> {
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(-0.25);
                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(170);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    })
                    .build();


            drive.followTrajectory(traj3);
            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj5 = drive.trajectorySequenceBuilder(poseEstimate)

                    .waitSeconds(0.7)
                    .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                    .addTemporalMarker(0, () -> {
                        leftClaw.setPosition(0.6);
                        rightClaw.setPosition(-0.1);

                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(400);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.5);
                    })

                    .build();
            drive.followTrajectorySequence(traj5);
            poseEstimate = drive.getPoseEstimate();
            Trajectory traj6 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0.6 , () -> {
                        leftLift.setTargetPosition(888);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(destX, destY , destH - Hchange))
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
                    .lineToLinearHeading(new Pose2d(49, -25 , 4.693))
                    .addTemporalMarker(0, () -> {
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(-0.25);
                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(150);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    })
                    .build();


            drive.followTrajectory(traj8);
            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj9 = drive.trajectorySequenceBuilder(poseEstimate)

                    .waitSeconds(0.7)
                    .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                    .addTemporalMarker(0, () -> {
                        leftClaw.setPosition(0.6);
                        rightClaw.setPosition(-0.1 );

                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(400);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.5);
                    })

                    .build();
            drive.followTrajectorySequence(traj9);
            poseEstimate = drive.getPoseEstimate();
            Trajectory traj10 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0.6 , () -> {
                        leftLift.setTargetPosition(888);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(destX, destY , destH - Hchange))
                    .build();
            drive.followTrajectory(traj10);

            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj11 = drive.trajectorySequenceBuilder(poseEstimate)
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
            drive.followTrajectorySequence(traj11);


            poseEstimate = drive.getPoseEstimate();
            //4th cone
            /**Trajectory traj13 = drive.trajectoryBuilder(poseEstimate)
                    .lineToLinearHeading(new Pose2d(51.196, -26.8 , 4.737))
                    .addTemporalMarker(0, () -> {
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.25);
                    })

                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(105);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    })
                    .build();


            drive.followTrajectory(traj13);
            poseEstimate = drive.getPoseEstimate();
            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj15 = drive.trajectorySequenceBuilder(poseEstimate)
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
            drive.followTrajectorySequence(traj15);
            poseEstimate = drive.getPoseEstimate();
            Trajectory traj16 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0.6 , () -> {
                        leftLift.setTargetPosition(888);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(destX, destY , destH - Hchange))
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
            drive.followTrajectorySequence(traj17);**/
        }
        else{
            //.lineToLinearHeading(new Pose2d(51.64, -3.585 , 4.711 ))

            Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0, () -> {
                        leftClaw.setPosition(0.5);
                        rightClaw.setPosition(0);
                    })
                    .lineToLinearHeading(new Pose2d(50.574049349703046, -4.281011379928819 , Math.toRadians(48.61271016484739)))
                    .addTemporalMarker(1.5, () -> {
                        leftLift.setTargetPosition(888);
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
         /**   Trajectory traj3 = drive.trajectoryBuilder(poseEstimate)
                    .lineToLinearHeading(new Pose2d(49, -26.8 , 4.693))
                    .addTemporalMarker(0, () -> {
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(-0.25);
                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(170);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    })
                    .build();


            drive.followTrajectory(traj3);
            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj5 = drive.trajectorySequenceBuilder(poseEstimate)

                    .waitSeconds(0.7)
                    .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                    .addTemporalMarker(0, () -> {
                        leftClaw.setPosition(0.6);
                        rightClaw.setPosition(-0.1);

                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(400);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.5);
                    })

                    .build();
            drive.followTrajectorySequence(traj5);
            poseEstimate = drive.getPoseEstimate();
            Trajectory traj6 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0.6 , () -> {
                        leftLift.setTargetPosition(888);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(destX, destY , destH - Hchange))
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
                    .lineToLinearHeading(new Pose2d(49, -26.8 , 4.693))
                    .addTemporalMarker(0, () -> {
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(-0.25);
                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(150);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    })
                    .build();


            drive.followTrajectory(traj8);
            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj9 = drive.trajectorySequenceBuilder(poseEstimate)

                    .waitSeconds(0.7)
                    .lineToLinearHeading(new Pose2d(47, -1.95 , 4.67426 ))

                    .addTemporalMarker(0, () -> {
                        leftClaw.setPosition(0.6);
                        rightClaw.setPosition(-0.1 );

                    })
                    .addTemporalMarker(0.5, () -> {
                        leftLift.setTargetPosition(400);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.5);
                    })

                    .build();
            drive.followTrajectorySequence(traj9);
            poseEstimate = drive.getPoseEstimate();
            Trajectory traj10 = drive.trajectoryBuilder(poseEstimate)
                    .addTemporalMarker(0.6 , () -> {
                        leftLift.setTargetPosition(888);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(destX, destY , destH - Hchange))
                    .build();
            drive.followTrajectory(traj10);

            poseEstimate = drive.getPoseEstimate();
            TrajectorySequence traj11 = drive.trajectorySequenceBuilder(poseEstimate)
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
            drive.followTrajectorySequence(traj11);**/


            poseEstimate = drive.getPoseEstimate();
            //4th cone
            /**Trajectory traj13 = drive.trajectoryBuilder(poseEstimate)
             .lineToLinearHeading(new Pose2d(51.196, -26.8 , 4.737))
             .addTemporalMarker(0, () -> {
             leftLift.setTargetPosition(0);
             leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             leftLift.setPower(0.25);
             })

             .addTemporalMarker(0.5, () -> {
             leftLift.setTargetPosition(105);
             leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             leftLift.setPower(1);
             })
             .build();


             drive.followTrajectory(traj13);
             poseEstimate = drive.getPoseEstimate();
             poseEstimate = drive.getPoseEstimate();
             TrajectorySequence traj15 = drive.trajectorySequenceBuilder(poseEstimate)
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
             drive.followTrajectorySequence(traj15);
             poseEstimate = drive.getPoseEstimate();
             Trajectory traj16 = drive.trajectoryBuilder(poseEstimate)
             .addTemporalMarker(0.6 , () -> {
             leftLift.setTargetPosition(888);
             leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             leftLift.setPower(0.75);
             })
             .lineToLinearHeading(new Pose2d(destX, destY , destH - Hchange))
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
             drive.followTrajectorySequence(traj17);**/
            poseEstimate = drive.getPoseEstimate();
            if (tagOfInterest.id == 1){ //left
                Trajectory left = drive.trajectoryBuilder(poseEstimate)
                        .addTemporalMarker(0, () -> {
                            leftLift.setTargetPosition(0);
                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(-0.25);
                        })
                        .lineToLinearHeading(new Pose2d(50.4, 18.87 , 3.13))
                        .build();
                drive.followTrajectory(left);
            } else if (tagOfInterest.id == 2){ //right
                Trajectory middle = drive.trajectoryBuilder(poseEstimate)
                        .addTemporalMarker(0, () -> {
                            leftLift.setTargetPosition(0);
                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(-0.25);
                        })
                        .lineToLinearHeading(new Pose2d(50.4, -5.75 , 3.13))
                        .build();
                drive.followTrajectory(middle);

            } else if (tagOfInterest.id == 3){//middle
                Trajectory right = drive.trajectoryBuilder(poseEstimate)
                        .addTemporalMarker(0, () -> {
                            leftLift.setTargetPosition(0);
                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(-0.25);
                        })
                        .lineToLinearHeading(new Pose2d(52.38, -26.65  , 4.64))
                        .build();
                drive.followTrajectory(right);

            }
        }






        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {

            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}