package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    int[] lastEncoders = {0,0,0};
    int[] encoders = {0,0,0};
    int[] encodersVel = {0,0,0};
    double lastHeading = 0;
    double odoHeading = 0;
    double offsetHeading = 0;
    double ticksToInches = 133000.0/72.0;
    Pose2d currentPose;
    double x = 0;
    double y = 0;
    Pose2d currentRobotVel;
    Pose2d lastRobotVel = new Pose2d(0,0,0);

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
    public void setEncodersVel (int[] arr){
        for (int i = 0; i < 3; i ++){
            encodersVel[i] = arr[i];
        }
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return currentRobotVel;
    }

    @Override
    public void update() {
        double fwrdSpeed = (encodersVel[0] + encodersVel[1])/(2.0*ticksToInches);
        double headSpeed = (encodersVel[0] - encodersVel[1])/(ticksToInches*(13.443402782));
        double horSpeed = (encodersVel[2]/ticksToInches + headSpeed*6.25);

        double deltaRight = encoders[0] - lastEncoders[0];
        double deltaLeft = encoders[1] - lastEncoders[1];
        double deltaHorizontal = encoders[2] - lastEncoders[2];
        double relDeltaX = (deltaLeft + deltaRight)/(2.0*ticksToInches);
        odoHeading = (encoders[0] - encoders[1])/(ticksToInches*(13.443402782)); //(encoders[0] - encoders[1])/(ticksToInches*(15.625))*1.162280135
        double heading = odoHeading + offsetHeading;

        currentRobotVel = new Pose2d(Math.cos(heading)*fwrdSpeed - Math.sin(heading)*horSpeed,Math.cos(heading)*horSpeed + Math.sin(heading)*fwrdSpeed,headSpeed);



        double deltaHeading = heading - lastHeading;
        double relDeltaY = deltaHorizontal/ticksToInches + deltaHeading*6.25;
        double simDeltaHeading = (heading - lastHeading)/50.0;
        double simHeading = lastHeading + simDeltaHeading/2;
        double numLoops = 50;

        for (int i = 0; i < numLoops; i ++) {
            x += Math.cos(simHeading) * relDeltaX/numLoops - Math.sin(simHeading) * relDeltaY/numLoops;
            y += Math.sin(simHeading) * relDeltaX/numLoops + Math.cos(simHeading) * relDeltaY/numLoops;
            simHeading += simDeltaHeading;
        }


        /*
        double deltaVelHeading = currentRobotVel.getHeading() - lastRobotVel.getHeading();
        double deltaVelX = currentRobotVel.getX() - lastRobotVel.getX();
        double deltaVelY = currentRobotVel.getY() - lastRobotVel.getY();
        for (int i = 0; i < numLoops; i ++) {
            double percentX = (deltaVelX/(2.0*Math.pow(numLoops,2) + deltaVelX*(double)(i)/Math.pow(numLoops,2)) + lastRobotVel.getX()/numLoops)/(lastRobotVel.getX()+deltaVelX/2.0);
            double percentY = (deltaVelY/(2.0*Math.pow(numLoops,2) + deltaVelY*(double)(i)/Math.pow(numLoops,2)) + lastRobotVel.getY()/numLoops)/(lastRobotVel.getY()+deltaVelY/2.0);
            double percentHed = (deltaVelHeading/(2.0*Math.pow(numLoops,2) + deltaVelHeading*(double)(i)/Math.pow(numLoops,2)) + lastRobotVel.getHeading()/numLoops)/(lastRobotVel.getHeading()+deltaVelHeading/2.0);
            simHeading += percentHed*deltaHeading;
            x += Math.cos(simHeading) * relDeltaX * percentX - Math.sin(simHeading) * relDeltaY * percentY;
            y += Math.sin(simHeading) * relDeltaX * percentX + Math.cos(simHeading) * relDeltaY * percentY;
        }
        */

        lastHeading = heading;
        for (int i = 0; i < 3; i ++) {
            lastEncoders[i] = encoders[i];
        }

        lastRobotVel = new Pose2d(currentRobotVel.getX(),currentRobotVel.getY(),currentRobotVel.getHeading());
        currentPose = new Pose2d(x,y,heading);
    }
}
