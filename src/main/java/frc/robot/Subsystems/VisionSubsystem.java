package frc.robot.Subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera camera;
    private boolean hasTarget;
    private PhotonPipelineResult result;
    private final RobotPoseEstimator poseEstimator;

    //to do: sendable chooser to select alliance so origin of robot can be set correctly
    //default origin is at the bottom right of blue alliance wall
    private static final AprilTagFieldLayout layout = new AprilTagFieldLayout(
        new ArrayList<AprilTag>() {
            {
                add(createTag(1, 7.05, 0.995, 0.665, 180));
                add(createTag(2, 7.05, 2.75, 0.665, 180));
                add(createTag(3, 7.05, 4.58, 0.665, 180));
                add(createTag(4, 7.66, 6.3, 0.435, 180));
                add(createTag(5, 0.36, 6.3, 0.435, 0));
                add(createTag(6, 0.97, 4.58, 0.665, 0));
                add(createTag(7, 0.97, 2.75, 0.665, 0));
                add(createTag(8, 0.97, 0.995, 0.665, 0));
            }
        },
        Constants.AutoConstants.FIELD_LENGTH,
        Constants.AutoConstants.FIELD_WIDTH
    );

    public VisionSubsystem() {
        camera = new PhotonCamera("lol");

        List<Pair<PhotonCamera, Transform3d>> camPair = new ArrayList<>();
        camPair.add(new Pair<>(camera, new Transform3d()));
        poseEstimator =
            new RobotPoseEstimator(
                layout,
                RobotPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
                camPair
            );
    }

    public Optional<Pair<Pose3d, Double>> getEstimatedPose(Pose2d prevPose) {
        poseEstimator.setReferencePose(prevPose);
        return poseEstimator.update();
    }

    //to do: swerve drive pose estimator object should be constructed in drivetrain subsystem
    public void updateOdometry(SwerveDrivePoseEstimator odometry) {
        if (!hasTarget) {
            return;
        }

        Optional<Pair<Pose3d, Double>> poseResult = getEstimatedPose(
            odometry.getEstimatedPosition()
        );
        if (poseResult.isPresent()) {
            poseResult.get();
            SmartDashboard.putString("robot pose", poseResult.get().toString());
        }
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
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return target;
            }
        }
        return null;
    }

    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
            return result.getBestTarget();
        } else {
            return null;
        }
    }

    public boolean getHasTarget() {
        return hasTarget;
    }

    public PhotonCamera getCam() {
        return camera;
    }

    public static AprilTag createTag(
        int id,
        double xPos,
        double yPos,
        double zPos,
        double angle
    ) {
        return new AprilTag(
            id,
            new Pose3d(
                xPos,
                yPos,
                zPos,
                new Rotation3d(0, 0, Math.toRadians(angle))
            )
        );
    }
}
