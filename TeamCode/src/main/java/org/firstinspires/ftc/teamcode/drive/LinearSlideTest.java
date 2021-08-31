package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LinearSlideTest", group="Iterative Opmode")

public class LinearSlideTest extends OpMode{
    // Declare OpMode members.
    public int position = 0;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.slidesLift.update(gamepad1.y);
        drive.update();
        telemetry.addData("Linear Slides Motor Position", drive.slidesEncoder);
        telemetry.addData("Toggle State", drive.slidesLift.getToggleState());
        telemetry.update();
    }

}