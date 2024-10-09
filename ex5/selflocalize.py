import os
import sys
import time
from timeit import default_timer as timer

import cv2
import numpy as np
import particle

import camera


sys.path.append(os.path.join(os.path.dirname(__file__), ".."))


from ex5 import staterobot
from localplanning_rrt.robot_models import PointMassModel
from localplanning_rrt.rrt import RRT
from localplanning_rrt.grid_occ import GridOccupancyMap



# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True  # Whether or not we are running on the Arlo robot


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
    You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


if isRunningOnArlo():
    import os
    import sys

    sys.path.append(os.path.join(os.path.dirname(__file__), ".."))


if isRunningOnArlo():
    try:
        #import calibrate
        import robot

        onRobot = True
    except (ImportError, AttributeError):
        onRobot = False
        print("selflocalize.py: robot module not present - forcing not running on Arlo!")


# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 4]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    4: (200.0, 0.0),  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN]  # Colors used when drawing the landmarks

goal = np.array([sum([x for x, y in landmarks.values()]) / 2, sum([y for x, y in landmarks.values()]) / 2])


def jet(x):
    """Colour map for drawing particles. This function determines the colour of
    a particle from its weight."""
    r = (
        (x >= 3.0 / 8.0 and x < 5.0 / 8.0) * (4.0 * x - 3.0 / 2.0)
        + (x >= 5.0 / 8.0 and x < 7.0 / 8.0)
        + (x >= 7.0 / 8.0) * (-4.0 * x + 9.0 / 2.0)
    )
    g = (
        (x >= 1.0 / 8.0 and x < 3.0 / 8.0) * (4.0 * x - 1.0 / 2.0)
        + (x >= 3.0 / 8.0 and x < 5.0 / 8.0)
        + (x >= 5.0 / 8.0 and x < 7.0 / 8.0) * (-4.0 * x + 7.0 / 2.0)
    )
    b = (
        (x < 1.0 / 8.0) * (4.0 * x + 1.0 / 2.0)
        + (x >= 1.0 / 8.0 and x < 3.0 / 8.0)
        + (x >= 3.0 / 8.0 and x < 5.0 / 8.0) * (-4.0 * x + 5.0 / 2.0)
    )

    return (255.0 * r, 255.0 * g, 255.0 * b)


def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE  # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x, y), 2, colour, 2)
        b = (
            int(particle.getX() + 15.0 * np.cos(particle.getTheta())) + offsetX,
            ymax - (int(particle.getY() + 15.0 * np.sin(particle.getTheta())) + offsetY),
        )
        cv2.line(world, (x, y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX()) + offsetX, ymax - (int(est_pose.getY()) + offsetY))
    b = (
        int(est_pose.getX() + 15.0 * np.cos(est_pose.getTheta())) + offsetX,
        ymax - (int(est_pose.getY() + 15.0 * np.sin(est_pose.getTheta())) + offsetY),
    )
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)


def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points.
        p = particle.Particle(
            600.0 * np.random.ranf() - 100.0,
            600.0 * np.random.ranf() - 250.0,
            np.mod(2.0 * np.pi * np.random.ranf(), 2.0 * np.pi),
            1.0 / num_particles,
        )
        particles.append(p)

    return particles


def calc_lengths_from_hypotenuse_and_angle(hypotenuse, angle):
    """
    Calculate the lengths of the two sides of a right triangle given the hypotenuse and one angle.

    This returns x distance and y distance.
    """
    return (hypotenuse * np.cos(angle), hypotenuse * np.sin(angle))


def gauss_dist(particle, dM: float, angle: float, deviation: float = 1):
    lx, ly = calc_lengths_from_hypotenuse_and_angle(dM, angle)
    di = np.sqrt((lx - particle.getX()) ** 2 + (ly - particle.getY()) ** 2)

    return (1 / (np.sqrt(2 * np.pi) * (deviation))) * np.exp(-(((dM - di) ** 2) / (2 * deviation)))


def gauss_angle(particle, dM: float, angle: float, deviation: float = 1):
    lx, ly = calc_lengths_from_hypotenuse_and_angle(dM, angle)
    theta = particle.getTheta()
    xi, yi = particle.getX(), particle.getY()
    di = np.sqrt((lx - xi) ** 2 + (ly - yi) ** 2)
    e_theta_i = np.array([np.cos(theta), np.sin(theta)])
    e_hat_theta_i = np.array([-np.sin(theta), np.cos(theta)])
    e_l_i = np.array([lx - xi, ly - yi]) / di
    phi_i = np.sign(np.dot(e_l_i, e_hat_theta_i)) * np.arccos(np.dot(e_l_i, e_theta_i))
    # print(f"{e_l_i=},{e_hat_theta_i=},{np.dot(e_l_i, e_hat_theta_i)=},{np.sign(np.dot(e_l_i, e_hat_theta_i))},{e_theta_i},{np.dot(e_l_i, e_theta_i)},{np.acos(np.dot(e_l_i, e_theta_i))}")
    # print(phi_i)
    return (1 / (np.sqrt(2 * np.pi) * (deviation))) * np.exp(
        -(((angle - phi_i) ** 2) / (2 * deviation))
    )


def get_weight(
    particle, dM: float, angle: float, dist_deviation: float = 1, angle_deviation: float = 1
):
    angle_gauss = gauss_angle(particle, dM, angle, angle_deviation)
    return gauss_dist(particle, dM, angle, dist_deviation) * angle_gauss


def resample(particles, num_particles):
    sorted_particles = sorted(particles, key=lambda x: x.getWeight())
    weights = np.array([p.getWeight() for p in sorted_particles])
    if weights.sum() != 0:
        normalized_weights = weights / sum(weights)

        resampled_particles = [
            sorted_particles[resampled_idx]
            for resampled_idx in np.random.choice(
                np.arange(num_particles), num_particles, p=normalized_weights
            )
        ]
    else:
        resampled_particles = sorted_particles

    for sorted_particle, resampled_particle in zip(sorted_particles, resampled_particles):
        sorted_particle.setX(resampled_particle.getX())
        sorted_particle.setY(resampled_particle.getY())
        sorted_particle.setTheta(resampled_particle.getTheta())
        sorted_particle.setWeight(resampled_particle.getWeight())


# Main program #
try:
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)

    # Initialize particles
    num_particles = 1000
    particles = initialize_particles(num_particles)

    est_pose = particle.estimate_pose(particles)  # The estimate of the robots current pose

    # Driving parameters
    velocity = 0.0  # cm/sec
    angular_velocity = 0.0  # radians/sec
    current_angle = np.pi / 2

    if onRobot:
        arlo = robot.Robot()
        robot_state = staterobot.StateRobot(2, arlo, particle.move_particle, particles)
        robot_model = PointMassModel([-0.1,0.1])
    else:
        arlo = None

    # Allocate space for world map
    world = np.zeros((500, 500, 3), dtype=np.uint8)

    # Draw map
    draw_world(est_pose, particles, world)

    print("Opening and initializing camera")
    if isRunningOnArlo():
        # cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = camera.Camera(0, robottype="arlo", useCaptureThread=False)
    else:
        # cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(0, robottype="macbookpro", useCaptureThread=False)

    while True:
        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord("q"):  # Quit
            break

        if not isRunningOnArlo():
            

            print("pathmotherfucker", path)

            if action == ord("w"):  # Forward
                velocity += 4.0
            elif action == ord("x"):  # Backwards
                velocity -= 4.0
            elif action == ord("s"):  # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord("a"):  # Left
                angular_velocity += 0.2
            elif action == ord("d"):  # Right
                angular_velocity -= 0.2
        else:
            map_ = GridOccupancyMap((0,0),(500, 500), 10)
            map_.grid = world
            rrt = RRT(
                start=[est_pose.getX(), est_pose.getY()],
                goal=goal,
                robot_model=robot_model,
                map=map_,
            )
            path = rrt.planning(animation=False)

            print("pathmotherfucker", path)
            if path is not None:
                path_from_start = path[::-1]
                robot_state.setAngle(est_pose.getTheta())
                robot_state.setPos((est_pose.getX(), est_pose.getY()))
                robot_state.setCurrentPath(path_from_start)
                while not robot_state.run_loop():
                    pass

        for parti in particles:
            theta = parti.getTheta()
            # unit vector pointing in the direction of the particle
            heading = np.array([np.cos(theta), np.sin(theta)])
            # scale with velocity
            deltaXY = heading * velocity
            # do the update
            particle.move_particle(parti, deltaXY[0], deltaXY[1], angular_velocity)

        # Use motor controls to update particles
        # XXX: Make the robot drive
        # XXX: You do this
        

        # Fetch next frame
        colour = cam.get_next_frame()

        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(objectIDs, type(None)):
            # List detected objects
            id_to_dist_n_angles = {}
            for i in range(len(objectIDs)):
                # print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
                id_to_dist_n_angles.setdefault(objectIDs[i], (np.inf, np.inf))
                exist_dist, exist_angle = id_to_dist_n_angles[objectIDs[i]]
                if dists[i] < exist_dist:
                    id_to_dist_n_angles[objectIDs[i]] = (dists[i], angles[i])
                # XXX: Do something for each detected object - remember, the same ID may appear several times

            # Compute particle weights
            # XXX: You do this
            # Add uncertainty line 4 of MCL algo
            particle.add_uncertainty(particles, 10, 0.1)
            for p in particles:
                w = 1
                for obj_id, (dM, angle) in id_to_dist_n_angles.items():
                    w *= get_weight(p, dM, angle, dist_deviation=10, angle_deviation=0.1)
                p.setWeight(w)
            # Resampling
            # XXX: You do this
            resample(particles, num_particles)

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0 / num_particles)

        est_pose = particle.estimate_pose(particles)  # The estimate of the robots current pose

        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)

            # Show frame
            cv2.imshow(WIN_RF1, colour)

            # Show world
            cv2.imshow(WIN_World, world)


finally:
    # Make sure to clean up even if an exception occurred

    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()
