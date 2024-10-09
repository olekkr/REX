import os
import sys
import time
from copy import copy
from timeit import default_timer as timer

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import cv2
import numpy as np
import particle

# own imports:
# import scipy.stats as stats
from numpy import random

import camera
from ex5 import command

# randomness:
rng = random.default_rng()

# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True  # Whether or not we are running on the Arlo robot


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../../../../Arlo/python")


try:
    if isRunningOnArlo():
        import robot
        onRobot = True
    else:
        onRobot = False
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False




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
landmarkIDs = [3,7]
landmarks = {
    3: (0.0, 0.0),  # Coordinates for landmark 1
    7: (100.0, 0.0)  # Coordinates for landmark 2
}
goal = np.array([50.0, 0.])


landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks

def normal(x, mu, sigma):
    return np.exp((mu-x)**2/(2.0*sigma**2))/np.sqrt(2*np.pi*sigma**2)

def particle_likelihood(particle, measurements):
    acc_likelihood = 1
    for l_id, (m_dist, m_ang) in measurements.items():
        # if observed l_id does not have known location - ignore ...
        if not l_id in landmarks.keys():
            print(f"alert:{l_id} seen and ignored")
            continue  

        part_pos = np.array([particle.getX(), particle.getY()])
        land_pos = landmarks[l_id]

        # the distance of particle from landmark
        particle_dist = np.linalg.norm(part_pos - np.array(land_pos))
        likelihood = normal(particle_dist - m_dist, 0, 10)
        # angle from particle to landmark
        particle_e =  np.array([np.cos(particle.getTheta()), np.sin(particle.getTheta())])
        landmark_e = (land_pos - part_pos)/particle_dist
        particle_theta = np.sign(np.dot(particle_e, landmark_e))*np.arccos(np.dot(particle_e, landmark_e))
        
        likelihood *= normal(particle_theta - m_ang, 0, 1)

        # accumulate multiplicatively for every landmark
        acc_likelihood *= likelihood
    return acc_likelihood

def resample_particles(particles, num_particles):
    pmf = np.zeros(len(particles))
    for i, p in enumerate(particles):
        pmf[i] = p.getWeight()
    choices = rng.choice(len(particles), num_particles, p=pmf)
    new_arr = []
    for i in choices:
        new_arr.append(copy(particles[i]))
    return new_arr
    
    


def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
                                 ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)



def initialize_particles(num_particles):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles


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

    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec

    # Initialize the robot (XXX: You do this)
    if onRobot:
        arlo = robot.Robot()
        try_goto_goal = False
    else:
        arlo = None

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    draw_world(est_pose, particles, world)

    print("Opening and initializing camera")
    if isRunningOnArlo():
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = camera.Camera(0, robottype='arlo', useCaptureThread=False)
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=False)

    while True:

        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
            elif action == ord('x'): # Backwards
                velocity -= 4.0
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
            elif action == ord('d'): # Right
                angular_velocity -= 0.2



        
        # Use motor controls to update particles
        # XXX: Make the robot drive
        # XXX: You do this
        for parti in particles:
            theta = parti.getTheta()
            # unit vector pointing in the direction of the particle 
            heading =  np.array([np.cos(theta), np.sin(theta)])
            # scale with velocity
            deltaXY = heading * velocity

            # do the update
            particle.move_particle(parti, deltaXY[0], deltaXY[1], angular_velocity )
        particle.add_uncertainty(particles, 10, 0.1)

        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(objectIDs, type(None)):
            measurement = dict() 
            for i in range(len(objectIDs)):
                if objectIDs[i] in measurement:
                      measurement[objectIDs[i]] += np.array([dists[i], angles[i]])/2
                      # DISCLAIMER: if there exists more than 2 observations of the same objID the result is not the mean.
                else: 
                    measurement[objectIDs[i]] = np.array([dists[i], angles[i]])


            # Compute particle weights
            for i, part in enumerate(particles):
                part.setWeight(particle_likelihood(part, measurement))
            weights = np.array([part.getWeight() ])
            
            # normalization step
            if sum(weights) != 0:
                weights /= sum(weights)
            for part, w in zip(particles, weights):
                part.setWeight(w)



            # Resampling
            # XXX: You do this
            particles = resample_particles(particles, num_particles)

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            # No observation - reset weights to uniform distribution
            particle.add_uncertainty(particles, 10, 0.1)            
            for p in particles:
                p.setWeight(1.0/num_particles)

    
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
        est_position = np.array([est_pose.x, est_pose.y])

        distance = np.linalg.norm(est_position - goal)
        angle = np.arccos(np.dot((est_position / (sum(est_position))), goal / (sum(goal))))

        print(f"ang:{angle}, dist: {distance}")         
        current_command = command.Command(arlo, distance, angle)
        current_command.update_command_state()


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

