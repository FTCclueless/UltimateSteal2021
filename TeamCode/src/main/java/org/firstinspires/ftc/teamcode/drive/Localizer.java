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
    Pose2d currentPose;
    double x = 0;
    double y = 0;
    double lastX = 0;
    double lastY = 0;
    Pose2d currentVel = new Pose2d(0,0,0);
    Pose2d currentRelVel = new Pose2d(0,0,0);
    Pose2d lastRelVel = new Pose2d(0,0,0);
    long lastTime = System.nanoTime();

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
        offsetHeading = (offsetHeading)*0.1 + headingDif*0.9;

    }

    public void setEncoders (int[] arr){
        for (int i = 0; i < 3; i ++){
            encoders[i] = arr[i];
        }
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {

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
        double relDeltaX = (deltaLeft + deltaRight)/(2.0*ticksToInches);
        odoHeading = (encoders[0] - encoders[1])/(ticksToInches*(13.443402782)); //(encoders[0] - encoders[1])/(ticksToInches*(15.625))*1.162280135
        double heading = odoHeading + offsetHeading;
        double deltaHeading = heading - lastHeading;
        double relDeltaY = deltaHorizontal/ticksToInches + deltaHeading*6.25;


        double relVelX = relDeltaX/loopTime;
        double relVelY = relDeltaY/loopTime;
        double relVelHeading = deltaHeading/loopTime;

        lastRelVel = new Pose2d(currentRelVel.getX(),currentRelVel.getY(),currentRelVel.getHeading());
        currentRelVel = new Pose2d(relVelX,relVelY,relVelHeading);


        /*
        double simDeltaHeading = (heading - lastHeading)/50.0;
        for (int i = 0; i < numLoops; i ++) {
            x += Math.cos(simHeading) * relDeltaX/numLoops - Math.sin(simHeading) * relDeltaY/numLoops;
            y += Math.sin(simHeading) * relDeltaX/numLoops + Math.cos(simHeading) * relDeltaY/numLoops;
            simHeading += simDeltaHeading;
        }
        */

        double simHeading = lastHeading;
        double numLoops = 50;
        double deltaVelHeading = currentRelVel.getHeading() - lastRelVel.getHeading();
        double deltaVelX = currentRelVel.getX() - lastRelVel.getX();
        double deltaVelY = currentRelVel.getY() - lastRelVel.getY();
        double t2 = Math.pow(numLoops,2);
        for (int i = 0; i < numLoops; i ++) {
            double pos = ((double)i)/t2;
            double percentX =   (deltaVelX/(2.0*t2)       + deltaVelX * pos       + lastRelVel.getX()/numLoops)      /(lastRelVel.getX()+deltaVelX/2.0);
            double percentY =   (deltaVelY/(2.0*t2)       + deltaVelY * pos       + lastRelVel.getY()/numLoops)      /(lastRelVel.getY()+deltaVelY/2.0);
            double percentHed = (deltaVelHeading/(2.0*t2) + deltaVelHeading * pos + lastRelVel.getHeading()/numLoops)/(lastRelVel.getHeading()+deltaVelHeading/2.0);
            //robot doesn't know how to do L'hoptial
            if ((lastRelVel.getY()+deltaVelY/2.0) == 0){percentX = 1/numLoops;}
            if ((lastRelVel.getX()+deltaVelX/2.0) == 0){percentX = 1/numLoops;}
            if ((lastRelVel.getHeading()+deltaVelHeading/2.0) == 0){percentX = 1/numLoops;}
            simHeading += percentHed*deltaHeading/2.0;
            x += Math.cos(simHeading) * (relDeltaX * percentX) - Math.sin(simHeading) * (relDeltaY * percentY);
            y += Math.sin(simHeading) * (relDeltaX * percentX) + Math.cos(simHeading) * (relDeltaY * percentY);
            simHeading += percentHed*deltaHeading/2.0;
        }

        currentVel = new Pose2d((x-lastX)/loopTime,(y-lastY)/loopTime,relVelHeading);

        lastX = x;
        lastY = y;
        lastHeading = heading;
        for (int i = 0; i < 3; i ++) {
            lastEncoders[i] = encoders[i];
        }
        currentPose = new Pose2d(x,y,heading);
    }
}
