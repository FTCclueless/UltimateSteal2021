package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    int[] lastEncoders = {0,0,0};
    int[] encoders = {0,0,0};
    double lastHeading = 0;
    double odoHeading = 0;
    double offsetHeading = 0;
    double ticksToInches = 133000.0/72.0;
    Pose2d currentPose = new Pose2d(0,0,0);
    double x = 0;
    double y = 0;
    double lastX = 0;
    double lastY = 0;
    Pose2d currentVel = new Pose2d(0,0,0);
    Pose2d currentRelVel = new Pose2d(0,0,0);
    Pose2d lastRelVel = new Pose2d(0,0,0);
    long lastTime = System.nanoTime();
    double startHeadingOffset = 0;

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        update();
        return currentPose;
    }

    public void updateHeading(double imuHeading){
        double headingDif = imuHeading-odoHeading;
        while (headingDif > Math.toRadians(180)){
            headingDif -= Math.toRadians(360);
        }
        while (headingDif < Math.toRadians(-180)){
            headingDif += Math.toRadians(360);
        }
        //ToDo: Tune these weights
        //We are getting issues with jittering in motion feedback.
        //This might be due to imu lag.
        //We should consider tuning these weights
        offsetHeading = (offsetHeading)*0.1 + headingDif*0.9; // 0.1, 0.9

    }

    public void setEncoders (int[] arr){
        for (int i = 0; i < 3; i ++){
            encoders[i] = arr[i];
        }
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        x = pose2d.getX();
        y = pose2d.getY();
        lastX = x;
        lastY = y;
        startHeadingOffset = pose2d.getHeading() - currentPose.getHeading();
        lastHeading = pose2d.getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return currentVel;
    }

    @Override
    public void update() {

        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        double deltaRight = encoders[0] - lastEncoders[0];
        double deltaLeft = encoders[1] - lastEncoders[1];
        double deltaHorizontal = encoders[2] - lastEncoders[2];
        double relDeltaX = (deltaLeft + deltaRight)/(2.0*ticksToInches) * -1;
        odoHeading = (encoders[0] - encoders[1])/(ticksToInches*(13.443402782)); //(encoders[0] - encoders[1])/(ticksToInches*(15.625))*1.162280135
        double heading = odoHeading + offsetHeading + startHeadingOffset;
        double deltaHeading = heading - lastHeading;
        double relDeltaY = deltaHorizontal/ticksToInches + deltaHeading*6.25;

        double relVelX = relDeltaX/loopTime;
        double relVelY = relDeltaY/loopTime;
        double relVelHeading = deltaHeading/loopTime;
        currentRelVel = new Pose2d(relVelX,relVelY,relVelHeading);

        double numLoops = 50;
        double simHeading = lastHeading;

        double simDeltaHeading = (heading - lastHeading)/50.0;
        for (int i = 0; i < numLoops; i ++) {
            simHeading += simDeltaHeading/2;
            x += Math.cos(simHeading) * relDeltaX/numLoops - Math.sin(simHeading) * relDeltaY/numLoops;
            y += Math.sin(simHeading) * relDeltaX/numLoops + Math.cos(simHeading) * relDeltaY/numLoops;
            simHeading += simDeltaHeading/2;
        }

        double w = 0.25;
        double newVelX = ((relDeltaX)/loopTime - currentVel.getX())*w + currentVel.getX()*(1.0-w);//X-lastX
        double newVelY = ((relDeltaY)/loopTime - currentVel.getY())*w + currentVel.getY()*(1.0-w);//y-lastY
        if (Math.abs(newVelX) < 1){
            newVelX =0;
        }
        if (Math.abs(newVelY) < 1){
            newVelY =0;
        }
        currentVel = new Pose2d(newVelX,newVelY,relVelHeading);

        lastX = x;
        lastY = y;
        lastHeading = heading;
        for (int i = 0; i < 3; i ++) {
            lastEncoders[i] = encoders[i];
        }
        currentPose = new Pose2d(x,y,heading);
        lastRelVel = new Pose2d(relVelX,relVelY,relVelHeading);
    }
}
