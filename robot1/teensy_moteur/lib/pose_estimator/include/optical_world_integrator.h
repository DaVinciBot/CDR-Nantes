/*
 * Rebuilds a table-frame optical pose from robot-frame optical displacements.
 *
 * WHY: the optical sensor runs with its own rotation disabled (enRot = 0), so
 * that it reports what the EKF expects: motion in the ROBOT frame. The
 * complementary filter, moved over unchanged from the 2026 code, still expects
 * an absolute pose in the TABLE frame. This integrator bridges the two: each
 * robot-frame step is rotated by the heading of the robot and accumulated.
 *
 * The heading is the pose estimator's, not the sensor's own: one heading for
 * the whole estimate, where the 2026 code mixed the sensor's and the IMU's.
 *
 * Goes away with the rewrite of the complementary filter, which will read the
 * robot-frame velocity directly.
 *
 * Arduino-free: builds and is tested in the native environment.
 */

#ifndef OPTICAL_WORLD_INTEGRATOR_H
#define OPTICAL_WORLD_INTEGRATOR_H

namespace pose {

class OpticalWorldIntegrator {
public:
    /** @brief Sets the accumulated pose, mm. */
    void reset(double x_mm, double y_mm);

    /**
     * @brief Adds one optical step.
     *
     * @param dx_robot_mm Step along robot x, mm.
     * @param dy_robot_mm Step along robot y, mm.
     * @param theta_rad Heading of the robot over the step, rad.
     */
    void add(double dx_robot_mm, double dy_robot_mm, double theta_rad);

    double x_mm() const { return x_mm_; }
    double y_mm() const { return y_mm_; }

private:
    double x_mm_ = 0.0;
    double y_mm_ = 0.0;
};

}  // namespace pose

#endif  // OPTICAL_WORLD_INTEGRATOR_H
