package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class CluelessFeedForwardTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double power = 0.0;
        double loops = 0.0;
        int average = 0;
        double lastVel = 0.0;
        double lastTime = System.nanoTime();
        while (!isStopRequested() && power < 1.0) {
            double currentTime = System.nanoTime();
            double loopSpeed = (currentTime-lastTime)/1000000000.0;
            lastTime = currentTime;
            loops += 1.0;
            power += 0.02;
            drive.setMotorPowers(power,power,power,power);
            drive.update();
            double acceleration = Math.abs(drive.currentVelocity.getX()-lastVel)/loopSpeed;
            Log.e("robotSpeed",drive.currentVelocity.getX()+"   " + lastVel);
            lastVel = drive.currentVelocity.getX();
            double kV = power/acceleration;
            Log.e("kV",kV+"");
            Log.e("Accel",acceleration+"");
            average += (int)(kV*1000000000);
        }
        drive.setMotorPowers(0,0,0,0);
        while (!isStopRequested()){
            telemetry.addData("Average",average);
            telemetry.addData("loops",loops);
            telemetry.addData("Average kV",average/loops);
            telemetry.update();
        }
    }
}
