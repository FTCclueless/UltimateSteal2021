package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="IntakeTest", group="Iterative Opmode")

public class IntakeTest extends OpMode{
    // Declare OpMode members.
    SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.controlIntake.update(gamepad1.a);
        drive.update();

        telemetry.addData("Intake Speed", drive.intakeMotor.getPower());
        telemetry.addData("Toggle State", drive.controlIntake.getToggleState());
        telemetry.update();
    }
}