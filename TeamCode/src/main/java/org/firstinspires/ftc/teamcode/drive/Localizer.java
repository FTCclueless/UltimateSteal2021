package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    int[] lastEncoders = {0,0,0};
    int[] encoders = {0,0,0};
    int[] encodersVel = {0,0,0};
    int[] lastEncodersVel = {0,0,0};
    double lastHeading = 0;
    double ticksToInches = 133000.0/72.0;
    Pose2d currentPose;
    double x = 0;
    double y = 0;

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        update();
        return currentPose;
    }

    public void setEncoders (int[] arr){
        for (int i = 0; i < 3; i ++){
            encoders[i] = arr[i];
        }
        //encoders[0] *= -1;
        //encoders[2] *= -1;
    }
    public void setEncodersVel (int[] arr){
        for (int i = 0; i < 3; i ++){
            encodersVel[i] = arr[i];
        }
        //encodersVel[0] *= -1;
        //encodersVel[2] *= -1;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        double deltaRight = encoders[0] - lastEncoders[0];
        double deltaLeft = encoders[1] - lastEncoders[1];
        double deltaHorizontal = encoders[2] - lastEncoders[2];
        double relDeltaX = (deltaLeft + deltaRight)/(2.0*ticksToInches);
        double heading = (encoders[0] - encoders[1])/(ticksToInches*(15.625));
        double deltaHeading = heading - lastHeading;
        double relDeltaY = deltaHorizontal/ticksToInches + deltaHeading*6.25;
        double simDeltaHeading = (heading - lastHeading)/50.0;
        double simHeading = lastHeading + simDeltaHeading/2;
        double numLoops = 50;
        //x += Math.cos(heading) * relDeltaX - Math.sin(heading) * relDeltaY;
        //y += Math.sin(heading) * relDeltaX + Math.cos(heading) * relDeltaY;

        Log.e("pose data", "x " + x + " y " + y + " " + relDeltaX + " " + relDeltaY + " " + encoders[0] + " " + encoders[1] + " " + encoders[2]);

        for (int i = 0; i < numLoops; i ++) {
            x += Math.cos(simHeading) * relDeltaX/numLoops - Math.sin(simHeading) * relDeltaY/numLoops;
            y += Math.sin(simHeading) * relDeltaX/numLoops + Math.cos(simHeading) * relDeltaY/numLoops;
            simHeading += simDeltaHeading;
        }


        lastHeading = heading;
        for (int i = 0; i < 3; i ++) {
            lastEncoders[i] = encoders[i];
            lastEncodersVel[i] = encodersVel[i];
        }

        currentPose = new Pose2d(x,y,heading);
    }
}
