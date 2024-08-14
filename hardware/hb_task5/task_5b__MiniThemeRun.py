import rclpy
from function_points import FunctionPointGenerator
import numpy as np
from controller import HolonomicBotController
from deadbands import HolaBotConfigReader

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of SubpointGenerator
    reader = HolaBotConfigReader("config.yaml")

    def lissajous_curve(t):
        x = 200 * np.cos(t)
        y = 150 * np.sin(4 * t)
        return y, x

    # Create an instance of the FunctionPointGenerator
    lissajous_generator = FunctionPointGenerator(explicit_function=lissajous_curve, num_points=100, t_range=(0, 2*np.pi))

    num_points_per_range = 150
    num_ranges = 3

    # Generate points using the specified function
    points = lissajous_generator.generate_points(num_points_per_range, num_ranges)

    bot_subpoints_list = [points[i*num_points_per_range:(i+1)*num_points_per_range] for i in range(num_ranges)]

    reader.read_deadbands()
    deadbands = reader.get_deadbands()

    deadbands_status=False

    hb_node_list=[]
    for bot_id, bands in deadbands.items():
        if deadbands_status:
            hb_node_list.append(HolonomicBotController(robot_id=bot_id, goal_positions=bot_subpoints_list[bot_id-1], goal_positions_theta=0, deadbands=bands))
        else:
            hb_node_list.append(HolonomicBotController(robot_id=bot_id, goal_positions=bot_subpoints_list[bot_id-1], goal_positions_theta=0))
        
    for bot in hb_node_list:
        bot.create_timer(0.1, bot.control_robot)

    # Create a MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()

    # Add your nodes to the executor
    for bot in hb_node_list:
        executor.add_node(bot)

    # Spin the nodes in parallel
    executor.spin()

if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        pass