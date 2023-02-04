package frc.robot.Subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//to do: add field layout and locations of april tags

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    boolean hasTarget; 
    PhotonPipelineResult result;
    

    public VisionSubsystem() {
        camera = new PhotonCamera("photonPi"); 
       
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult(); 
        hasTarget = result.hasTargets(); 
        if (hasTarget) {
            this.result = result;
        }
    }

    public PhotonTrackedTarget getTargetWithID(int id) { 
        List<PhotonTrackedTarget> targets = result.getTargets(); 
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) {
                return i; 
            }
        }
        return null; 
    }
    
    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
        return result.getBestTarget(); 
        }
        else {
            return null; 
        }
    }

    public boolean getHasTarget() {
        return hasTarget; 
    }

    public PhotonCamera getCam() {
        return camera;
    }

}

