/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final PhotonCamera FrontCamera;
    private final PhotonCamera BackCamera;
    private final PhotonCamera LeftCamera;
    private final PhotonCamera RightCamera;
    
    private final PhotonPoseEstimator FrontPhotonEstimator;
    private final PhotonPoseEstimator BackPhotonEstimator;
    private final PhotonPoseEstimator LeftPhotonEstimator;
    private final PhotonPoseEstimator RightPhotonEstimator;

    private Matrix<N3, N1> curStdDevs;
    private final CommandSwerveDrivetrain estConsumer;

    // Simulation
    private PhotonCameraSim FrontCameraSim;
    private PhotonCameraSim BackCameraSim;
    private PhotonCameraSim LeftCameraSim;
    private PhotonCameraSim RightCameraSim;

    private VisionSystemSim visionSim;

    private Boolean visionUpdatesEnabled = true;

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(CommandSwerveDrivetrain estConsumer) {
        this.estConsumer = estConsumer;
        FrontCamera = new PhotonCamera(VisionConstants.kFrontPhoton);
        BackCamera = new PhotonCamera(VisionConstants.kBackPhoton);
        LeftCamera = new PhotonCamera(VisionConstants.kLeftPhoton);
        RightCamera = new PhotonCamera(VisionConstants.kRightPhoton);

        FrontPhotonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kFrontCamOffset);
        BackPhotonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kBackOffset);
        LeftPhotonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kLeftCamOffset);
        RightPhotonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRightCamOffset);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(VisionConstants.kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            FrontCameraSim = new PhotonCameraSim(FrontCamera, cameraProp);
            BackCameraSim = new PhotonCameraSim(BackCamera, cameraProp);
            LeftCameraSim = new PhotonCameraSim(LeftCamera, cameraProp);
            RightCameraSim = new PhotonCameraSim(RightCamera, cameraProp);

            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(FrontCameraSim, VisionConstants.kFrontCamOffset);
            visionSim.addCamera(BackCameraSim, VisionConstants.kBackOffset);
            visionSim.addCamera(LeftCameraSim, VisionConstants.kLeftCamOffset);
            visionSim.addCamera(RightCameraSim, VisionConstants.kRightCamOffset);

            //backCameraSim.enableDrawWireframe(true);
        }
    }
    
    @Override
    public void periodic() {
        if(visionUpdatesEnabled){
            processCamera(FrontCamera, FrontPhotonEstimator);
            //processCamera(BackCamera, BackPhotonEstimator);
            //processCamera(LeftCamera, LeftPhotonEstimator);
            //processCamera(RightCamera, RightPhotonEstimator);
        }
        SmartDashboard.putBoolean("Vision Enabled", visionUpdatesEnabled);
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimater) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimater.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 10));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void processCamera(PhotonCamera cam, PhotonPoseEstimator estimator){
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : cam.getAllUnreadResults()) {
            // Only process if we have targets
            if (!result.hasTargets()) continue;

            var bestTarget = result.getBestTarget();
            
            // --- AMBIGUITY FILTER ---
            if (bestTarget.getPoseAmbiguity() > VisionConstants.kPoseAmbiguity) continue;

            visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets(), estimator);

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        estConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs.times(2));
                    });
            SmartDashboard.putNumber("Stuff", result.getTimestampSeconds());
        }
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    // Commands
    public void toggleVisionUpdates(){
        visionUpdatesEnabled = !visionUpdatesEnabled;
    }
    public Command toggleVisionUpdatesC(){
        Command result = runOnce(()->toggleVisionUpdates());
        return result;
    }
}