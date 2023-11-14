package org.firstinspires.ftc.teamcode.drive.runnable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.runnable.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    protected DcMotor leftLift;
    protected Servo leftClaw, rightClaw, tentioner;
    public static double driveMultiplier = 0.7;
    public static double turnMiltiplier = 0.7;

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    // Variables for going to point
    static int INPUT_X = 0;
    static int INPUT_Y = 0;
    static boolean CANAUTO = true;





    @Override
    public void runOpMode() throws InterruptedException {
        tentioner = hardwareMap.servo.get("tentioner");

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean isPressed = false;
        tentioner.setPosition(0.05);

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            //   PoseStorage.currentPose = poseEstimate;
            telemetry.addData("leftClaw", leftClaw.getPosition());
            telemetry.addData("rightClaw", rightClaw.getPosition());
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("TargetX: ", INPUT_X);
            telemetry.addData("TargetY: " , INPUT_Y);
            telemetry.addData("ArmPos", leftLift.getCurrentPosition());
            telemetry.addData("Ispressed", isPressed);
            telemetry.update();



            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {


                //Gamepad 1
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * driveMultiplier,
                                    gamepad1.right_stick_x * turnMiltiplier,
                                    gamepad1.left_stick_x * driveMultiplier

                            )

                    );


                    //0 = pick up cone
                    //1 = Ground goal
                    //2 = short
                    //3 = medium
                    // 4 = high
                    // Set Arm for High goal
                    if (gamepad1.dpad_left){
                        if (leftLift.getCurrentPosition() > 200){


                            TrajectorySequence backwards = drive.trajectorySequenceBuilder(poseEstimate)
                                    .waitSeconds(0.2)
                                    .addTemporalMarker(0, () -> {
                                        leftLift.setTargetPosition(leftLift.getCurrentPosition() - 200);
                                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        leftLift.setPower(-0.25);
                                    })
                                    .back(2)

                                    .addTemporalMarker(0.2, () -> {
                                        leftLift.setTargetPosition(leftLift.getCurrentPosition() - 200);
                                        leftClaw.setPosition(0.25);
                                        rightClaw.setPosition(0.25);
                                    })
                                    //36.3 or 35.6
                                    .build();
                            drive.followTrajectorySequence(backwards);

                            leftLift.setTargetPosition(0);
                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(-0.25);
                        }
                    }
                    // Go for bottom
                    else if (gamepad1.dpad_right){ // cycle through cones
                        poseEstimate = new Pose2d(0, 0, Math.toRadians(90));
                        leftClaw.setPosition(0.6);
                        rightClaw.setPosition(-0.1);
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .back(27.39)
                                //36.3 or 35.6
                                .build();
                        drive.followTrajectory(traj);
                        drive.turn(Math.toRadians(-115));
                        poseEstimate = drive.getPoseEstimate();

                        leftLift.setTargetPosition(700);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(0.5);

                        Trajectory forwards = drive.trajectoryBuilder(poseEstimate)
                                .forward(2)
                                .build();
                        drive.followTrajectory(forwards);
                        poseEstimate = drive.getPoseEstimate();
                        TrajectorySequence wait = drive.trajectorySequenceBuilder(poseEstimate)
                                .waitSeconds(1)
                                .build();
                        drive.followTrajectorySequence(wait);
                        poseEstimate = drive.getPoseEstimate();
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(-0.25);
                        leftClaw.setPosition(0.25);
                        rightClaw.setPosition(0.25);
                        Trajectory back1 = drive.trajectoryBuilder(poseEstimate)
                                .back(2)
                                .build();
                        drive.followTrajectory(back1);
                        drive.turn(Math.toRadians(115));
                        poseEstimate = drive.getPoseEstimate();
                        Trajectory back2 = drive.trajectoryBuilder(poseEstimate)
                                .forward(27.39)
                                .build();
                        drive.followTrajectory(back2);
                    } else if (gamepad1.right_trigger > 0.05 || gamepad2.right_trigger > 0.05){
                        leftLift.setTargetPosition(0);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(-0.25);
                    }  else if (gamepad1.a || gamepad2.a){
                        leftLift.setTargetPosition(40);

                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    }  else if (gamepad1.b || gamepad2.b){

                        if (leftLift.getCurrentPosition() > 350){
                            leftLift.setTargetPosition(350);

                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(-0.25);
                        } else {
                            leftLift.setTargetPosition(350);
                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(1);
                        }

                    }else if (gamepad1.y || gamepad2.y){
                        if (leftLift.getCurrentPosition() > 580){
                            leftLift.setTargetPosition(580);

                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(-0.25);
                        } else {
                            leftLift.setTargetPosition(580);

                            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            leftLift.setPower(1);
                        }
                    }
                    if (gamepad1.left_trigger > 0.05 || gamepad2.left_trigger > 0.05){
                        leftLift.setTargetPosition(888);
                        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLift.setPower(1);
                    }

                   /* // Know when to stop
                    if (leftLift.getCurrentPosition() < 2){
                        leftLift.setPower(0);
                        isPressed = false;
                    } else if (leftLift.getCurrentPosition() == 40 && goingTo == 1){
                        leftLift.setPower(0.01);
                        isPressed = false;
                    } else if (leftLift.getCurrentPosition() == 290 && goingTo == 2){
                        leftLift.setPower(0.01);
                        isPressed = false;
                    } else if (leftLift.getCurrentPosition() == 470 && goingTo == 3){
                        leftLift.setPower(0.01);
                        isPressed = false;
                    } else if (leftLift.getCurrentPosition() == 700 && goingTo == 4) {
                        leftLift.setPower(0.01);
                        isPressed = false;
                    }

                    if (leftLift.getCurrentPosition() == 416 && goingTo == 4){
                        leftLift.setPower(0.1);
                    }*/



// Medium 416
                    // Low 241
                    if (gamepad1.left_bumper || gamepad2.left_bumper){ // close
                        leftClaw.setPosition(0.6);
                        rightClaw.setPosition(-0.1);

                    } else if (gamepad1.right_bumper || gamepad2.right_bumper){// open
                        leftClaw.setPosition(0.25);
                        rightClaw.setPosition(0.25);
                    } else if (gamepad1.dpad_up && !isPressed || gamepad2.dpad_up && !isPressed){
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        leftLift.setPower(0.5);
                        isPressed = true;

                    } else if (gamepad1.dpad_down && !isPressed || gamepad2.dpad_down && !isPressed){
                        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        leftLift.setPower(-0.25);
                        isPressed = true;
                    } else if (!gamepad1.dpad_up && !gamepad2.dpad_up && isPressed){
                        isPressed = false;
                        leftLift.setPower(0.1);
                    } else if (!gamepad1.dpad_down && !gamepad2.dpad_up && isPressed){
                        isPressed = false;
                        leftLift.setPower(0.1);
                    }

//Gamepad 2

                    break;
                case AUTOMATIC_CONTROL:
                    // If b is pressed, we break out of the automatic following

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + currentMode);
            }
        }
    }



    //Other Statements----------------------
    public static double distance(double x1, double y1, double x2, double y2){
        double x = Math.pow(x2-x1, 2);
        double y = Math.pow(y2 - y1, 2);
        return Math.sqrt(x + y);
    }
}