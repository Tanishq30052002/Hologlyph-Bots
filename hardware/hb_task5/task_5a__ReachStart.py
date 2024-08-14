import rclpy
import numpy as np
from subpoints import SubpointGenerator
from controller import HolonomicBotController
from deadbands import HolaBotConfigReader

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of SubpointGenerator
    subpoint_generator = SubpointGenerator(num_points_per_side=5)
    reader = HolaBotConfigReader("config.yaml")

    # Define vertices for each shape
    hexagon_vertices = np.array([[200, 150], [175, 200], [125, 200], [100, 150], [125, 100], [175, 100], [200, 150]])
    triangle_vertices = np.array([[300, 100], [400, 100], [350, 200], [300, 100]])
    rectangle_vertices = np.array([[200, 300], [400, 300], [400, 400], [200, 400], [200, 300]])

    # Generate subpoints for each shape
    hexagon_subpoints = subpoint_generator.generate_subpoints(vertices=hexagon_vertices)
    triangle_subpoints = subpoint_generator.generate_subpoints(vertices=triangle_vertices)
    rectangle_subpoints = subpoint_generator.generate_subpoints(vertices=rectangle_vertices)

    bot_subpoints_list = [hexagon_subpoints, triangle_subpoints, rectangle_subpoints]

    # Start Points
    bot_subpoints_list = [[bot_subpoints_list[i][0]] for i in range(len(bot_subpoints_list))]
    
    reader.read_deadbands()
    deadbands = reader.get_deadbands()

    hb_node_list=[]
    for bot_id, bands in deadbands.items():
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