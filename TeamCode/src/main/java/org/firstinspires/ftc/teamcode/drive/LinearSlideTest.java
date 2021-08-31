package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.drive.ButtonToggle;

@TeleOp(name="LinearSlideTest", group="Iterative Opmode")

public class LinearSlideTest extends OpMode{
    // Declare OpMode members.
    public int position = 0;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

//        linearSlideMotor = hardwareMap.get(DcMotor.class, "slide_motor_1");
//        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
//        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //driveTrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        drive.buttonLift.update(gamepad1.y);
        drive.update();
        telemetry.addData("Linear Slides Motor Position", drive.slidesEncoder);
        telemetry.addData("toggle state", drive.buttonLift.getToggleState());
        telemetry.update();
    }

}