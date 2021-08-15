package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    int[] encoders;
    int[] encodersVel;
    Pose2d currentPose;


    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        update();
        return currentPose;
    }

    public void setEncoders (int[] arr){
        encoders = arr;
    }
    public void setEncodersVel (int[] arr){
        encodersVel = arr;
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
        //For the encoders
        /*
        0 => left
        1 => right
        2 => horizontal
        */

    }
}
