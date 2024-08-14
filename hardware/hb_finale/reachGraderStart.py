import rclpy
from controller import HolonomicBotController
from configReader import HolaBotConfigReader

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of SubpointGenerator
    reader = HolaBotConfigReader("config.yaml")

    # Start Points
    bot_grader_start_points = [[[(397, 455)]], [[(243, 448)]], [[(93, 440)]]]

    reader.read_deadbands()
    deadbands = reader.get_deadbands()

    deadbands_status=True

    # Create a MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()

    hb_node_list=[]
    kpx, kpy, kpw= 4, 4, 50
    kix, kiy, kiw= 0, 0, 0
    kdx, kdy, kdw= 4, 4, 10
    kp={"x":kpx, "y":kpy, "w":kpw}
    ki={"x":kix, "y":kiy, "w":kiw}
    kd={"x":kdx, "y":kdy, "w":kdw}

    errors=[3, 0.5] #Linear, Theta

    for bot_id, bands in deadbands.items():
        if deadbands_status:
            hb_node_list.append(HolonomicBotController(robot_id=bot_id, initialize=True, goal_positions=bot_grader_start_points[bot_id-1], goal_positions_theta=0, errors=errors, kp=kp, ki=ki, kd=kd, deadbands=bands))
        else:
            hb_node_list.append(HolonomicBotController(robot_id=bot_id, initialize=True, goal_positions=bot_grader_start_points[bot_id-1], goal_positions_theta=0, errors=errors, kp=kp, ki=ki, kd=kd))

    for bot in hb_node_list:
        bot.timer = bot.create_timer(0.1, bot.control_robot)
        executor.add_node(bot)

    # Spin the nodes in parallel
    executor.spin()

if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        pass

# if __name__ == '__main__':
#     try:
#         main()

#     except KeyboardInterrupt:
#         pass