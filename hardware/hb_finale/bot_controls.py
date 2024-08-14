import rclpy
from pointsGenerator import FunctionPointGenerator, ImagePointGenerator
import numpy as np
from controller import HolonomicBotController
from configReader import HolaBotConfigReader
import argparse

# Define the curve function for function mode
def curve_function(t):
    """
    Curve function.

    Parameters:
        t (np.ndarray): Time values.

    Returns:
        tuple: Tuple of y and x values.
    """

    # Flower Function
    r = 220*np.cos(4*t)
    x = r*np.cos(t)
    y = r*np.sin(t)
    return y, x

# Main function to control the robots
def main(image_mode):
    """
    The main function to define the points to be given to the robot. This has two modes.
    Mode 1 - function_mode:
                In function mode the bot will have to follow some function given in terms of x and y, parametric equations.
    Mode 2 - image_mode:
                In image_mode the bots need to draw the image. So the points will be along the contours of the image. This mode will read these points from the provided text file and set these goal positions for the robots.
    Arguments:
        - image_mode: (bool) If true, this function will operate in image mode.
    """
    rclpy.init()

    # Create an instance of HolaBotConfigReader to read configuration
    reader = HolaBotConfigReader("config.yaml")

    bot_subpoints_list = None
    
    if image_mode:
        print("Image Mode Started....")
        # Create instances of ImagePointGenerator for each robot
        bot1 = ImagePointGenerator("bot1_points_elephant.txt", 10)
        bot2 = ImagePointGenerator("bot2_points_elephant.txt", 10)
        bot3 = ImagePointGenerator("bot3_points_elephant.txt", 10)

        # Generate points for each robot from the provided text files
        bot_subpoints_list = [bot3.generate_points(), bot2.generate_points(), bot1.generate_points()]
    else:
        print("Function Mode Started....")
        # Create an instance of the FunctionPointGenerator for generating points using the flower curve function
        function_generator = FunctionPointGenerator(explicit_function=curve_function, num_points=100, t_range=(0, 2*np.pi))

        num_points_per_range = 125
        num_ranges = 3

        # Generate points using the flower curve function
        points = function_generator.generate_points(num_points_per_range, num_ranges)
        bot_subpoints_list = [[np.concatenate((points[i*num_points_per_range:(i+1)*num_points_per_range], [points[(i+1) % num_ranges * num_points_per_range]]))] for i in range(num_ranges)]

    # Initialize starting positions for the robots
    bot_start_list = [[[bot_subpoints_list[i][0][0]]] for i in range(len(bot_subpoints_list))]

    # Read deadbands from configuration
    reader.read_deadbands()
    deadbands = reader.get_deadbands()
    deadbands_status = True

    # Create a MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()

    hb_node_list = []

    # PID controller parameters and Threshold Errors
    kpx, kpy, kpw = None, None, None
    kix, kiy, kiw = None, None, None
    kdx, kdy, kdw = None, None, None
    errors = None
    
    if image_mode:
        kpx, kpy, kpw= 6, 6, 100
        kix, kiy, kiw= 0, 0, 0
        kdx, kdy, kdw= 5, 5, 15
        errors = [5, 1]  # Linear, Theta

    else:
        kpx, kpy, kpw = 4, 4, 50
        kix, kiy, kiw = 0, 0, 0
        kdx, kdy, kdw = 4, 4, 10
        errors = [5, 1]  # Linear, Theta


    kp = {"x": kpx, "y": kpy, "w": kpw}
    ki = {"x": kix, "y": kiy, "w": kiw}
    kd = {"x": kdx, "y": kdy, "w": kdw}

    # Initialize controller nodes for each robot
    for bot_id, bands in deadbands.items():
        if deadbands_status:
            hb_node_list.append(HolonomicBotController(robot_id=bot_id, initialize=True, goal_positions=bot_start_list[bot_id-1], goal_positions_theta=0, errors=errors, kp=kp, ki=ki, kd=kd, deadbands=bands))
        else:
            hb_node_list.append(HolonomicBotController(robot_id=bot_id, initialize=True, goal_positions=bot_start_list[bot_id-1], goal_positions_theta=0, errors=errors, kp=kp, ki=ki, kd=kd))

    # Move the bots to start position sequentially
    for bot in hb_node_list:
        bot.timer = bot.create_timer(0.1, bot.control_robot)
        while not bot.reached_start:
            rclpy.spin_once(bot)
        bot.goal_positions_all = bot_subpoints_list[bot.robot_id-1]

    print("Bots Initialized")

    # Configure bots for movement
    for bot in hb_node_list:
        bot.current_contour = 0
        bot.initializing = False
        bot.timer = bot.create_timer(0.1, bot.control_robot)
        executor.add_node(bot)
                           
    # Spin the nodes in parallel
    executor.spin()

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Choose mode between function and image mode. Default mode is functional mode. If no flags passed, then functional mode will be followed. To make the figure in image mode use the flag --image_mode")
    parser.add_argument("--image_mode", action='store_true', help="Image mode flag to operate the function in image mode")
    args = parser.parse_args()
    try:
        # Call main function with the specified mode
        main(image_mode=args.image_mode)
    except KeyboardInterrupt:
        print("Keyboard Interrupt. Shutting down.")
        # If the program is interrupted using the keyboard, destroy all nodes 
        rclpy.shutdown()
        pass
