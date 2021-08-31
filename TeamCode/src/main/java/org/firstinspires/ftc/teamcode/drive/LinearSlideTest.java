package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.drive.ButtonToggle;

@TeleOp(name="Linear Slide Test", group="Iterative Opmode")

public class LinearSlideTest extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private ButtonToggle buttonY2 = new ButtonToggle();

    public int position = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

//        linearSlideMotor = hardwareMap.get(DcMotor.class, "slide_motor_1");
//        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
//        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive.linearSlidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //driveTrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        position = SampleMecanumDrive.linearSlidesMotor.getCurrentPosition();
        SampleMecanumDrive.linearSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if(buttonY2.toggled(true)) {
            if(position < 100 ) { //default val
                SampleMecanumDrive.linearSlidesMotor.setPower(1.0);
            }
            else {
                SampleMecanumDrive.linearSlidesMotor.setPower(0.0);
            }
        }

        SampleMecanumDrive.linearSlidesMotor.setPower(1.0);

        telemetry.addData("Linear Slides Motor Position", position);
    }

}