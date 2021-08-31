package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.ButtonToggle;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Controller 1
    private ButtonToggle button_rb = new ButtonToggle();

    //Controller 2

    private ButtonToggle buttonA2 = new ButtonToggle();
    private ButtonToggle buttonY2 = new ButtonToggle();

    private boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            double forward = gamepad1.left_stick_y * -0.75;
            double left = gamepad1.left_stick_x * 0.75;
            double turn = gamepad1.right_stick_x * 0.75;

            if(button_rb.toggled(gamepad1.right_bumper)) {
                slowMode = !slowMode;
            }

            if(slowMode == true){
                forward = gamepad1.left_stick_y * -0.45;
                left = gamepad1.left_stick_x * 0.45;
                turn = gamepad1.right_stick_x * 0.45;
            }

//            if(buttonA2.toggled(gamepad2.a)) {
//                if(buttonA2.toggleState()) {
//                    intakeMotor.setPower(1.0);
//                }
//                else {
//                    intakeMotor.setPower(0.0);
//                }
//            }

            if(buttonA2.toggled(gamepad2.a)) {
                    SampleMecanumDrive.intakeMotor.setPower(1.0);
                }
                else {
                    SampleMecanumDrive.intakeMotor.setPower(0.0);
                }

//            if(buttonY2.toggled(gamepad2.y)) {
//                if(buttonY2.toggleState()) {
//                    linearSlidesMotor.setPower(1.0);
//                }
//                else {
//                    linearSlidesMotor.setPower(0.0);
//                }
//            }

            if(buttonY2.toggled(gamepad2.y)) {
                SampleMecanumDrive.linearSlidesMotor.setPower(1.0);
            }
                else {
                    SampleMecanumDrive.linearSlidesMotor.setPower(0.0);
                }

            drive.setMotorPowers((forward+left+turn), (forward-left+turn), (forward+left-turn), (forward-left-turn));

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("speedX", drive.currentVelocity.getX());
            telemetry.addData("speedY", drive.currentVelocity.getY());
            telemetry.addData("speedHeading", drive.currentVelocity.getHeading());
            telemetry.update();

        }
    }
}
