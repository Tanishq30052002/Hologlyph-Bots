from hmac import new
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class ImagePointGenerator:
    """
    Class for extracting points from a very large list of contour points
    """
    def __init__(self, file_name, space=1):
        """
        Initialize the ImagePointGenerator.

        Parameters:
            file_name (str): The name of the image file.
            space (int): The space between the points.
        """
        self.file_name = file_name
        self.space = space
    
    def generate_points(self):
        """
        Extract every space points from the list of points in the txt file
        """
        with open(self.file_name, 'r') as file:
            """
            Extract the points which are present in the file.
            Take every self.space point from the file and append to the list of points_list

            We first read the file and then find the points in the file. There are clusters in the file
            where after some empty lines there are a lot of points for some time. We will first extract one cluster
            and then take every self.space point from the cluster and append to the list of points_list
            """
            all_points = []
            points = []
            lines = file.readlines()

            # Finding the cluster and then extract every self.space point from the cluster
            scanned_lines = 0
            while scanned_lines < len(lines):
                """
                Check if we have a cluster, if cluster we extract all points in the cluster and then
                take every self.space point from the cluster and append to the list of points_list.
                If empty line then we have reached the end of the cluster and we append the points to the list of points_list
                """
                if lines[scanned_lines] == '\n':
                    if len(points) > 0:
                        all_points.append(np.array(points))
                    points = []
                    scanned_lines += 1
                    continue
                else:
                    """
                    Run a loop to fill a temporary list with all the values till the next empty line. Then we 
                    take every self.space point from the temporary list and append to the list of points_list
                    """
                    temp_points = []
                    while scanned_lines < len(lines) and lines[scanned_lines] != '\n':
                        point = np.fromstring(lines[scanned_lines][2:-3], dtype=int, sep=', ')
                        temp_points.append(point)
                        scanned_lines += 1

                    for i in range(0, len(temp_points), self.space):
                        points.append(temp_points[i])
                    
            if len(points) > 0:
                all_points.append(np.array(points))



        return all_points
            

class ImagePointViewer:
    """
    Class for visualizing points in a video using OpenCV.
    """
    def __init__(self, bot_points, colors, video_size=(500, 500)):
        """
        Initialize the FunctionPointVisualizer.

        Parameters:
            bot_points (list): List of arrays containing points for each bot.
            video_size (tuple): Size of the video window (width, height).
        """
        self.bot_points = bot_points
        self.video_size = video_size
        self.colors = colors

    def visualize_points(self):
        """
        Visualize the points in a frame.
        """
        # Create a black image
        frame = np.zeros((self.video_size[1], self.video_size[0], 3), np.uint8)

        # Draw the points
        for bot_id in range(len(self.bot_points)):
            color = self.colors[bot_id]
            bot_groups = self.bot_points[bot_id]

            last_point = None

            for bot_group in bot_groups:

                for points in bot_group:
                    x, y = points 
                    cv2.circle(frame, (x, y), 1, color, -1)

                    if last_point is not None:
                        cv2.line(frame, last_point, (x, y), color, 1)
                    
                    last_point = (x, y)
                
        # Display the frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)

class PointVisualizer:
    """
    Class for visualizing points in a video using OpenCV.
    """
    def __init__(self, bot_points, colors, video_size=(500, 500)):
        """
        Initialize the FunctionPointVisualizer.

        Parameters:
            bot_points (list): List of arrays containing points for each bot.
            video_size (tuple): Size of the video window (width, height).
        """
        self.bot_points = bot_points
        self.video_size = video_size
        self.colors = colors

    def visualize_points(self):
        """
        Visualize the bot points in a video using OpenCV.
        """
        max_points = 0
        for bot_points in self.bot_points:
            max_points = max(max_points, len(bot_points))
        i = 0

        while i < max_points:
            image = np.zeros((self.video_size[1], self.video_size[0], 3), dtype=np.uint8)

            for bot_idx, bot_color in enumerate(self.colors):
                bot_points = self.bot_points[bot_idx]

                for k in range(min(i + 1, len(bot_points))):
                    cv2.circle(image, tuple(bot_points[k]), 5, bot_color, -1)

                    if k > 0:
                        cv2.line(image, tuple(bot_points[k-1]), tuple(bot_points[k]), bot_color, 2)

            cv2.imshow('Points Visualizer', image)

            key = cv2.waitKey(1)
            if key == 27:  # 'Esc' key
                break

            i += 1

        cv2.waitKey(0)
        cv2.destroyAllWindows()

class FunctionPointGenerator:
    """
    Class for generating points based on an explicit function.
    """
    def __init__(self, explicit_function, num_points=100, t_range=(0, 2*np.pi)):
        """
        Initialize the FunctionPointGenerator.

        Parameters:
            explicit_function (function): The explicit function to generate points.
            num_points (int): Number of points to generate.
            t_range (tuple): Time range for the function.
        """
        self.explicit_function = explicit_function
        self.num_points = num_points
        self.t_range = t_range

    def generate_points(self, num_points_per_range=20, num_ranges=3):
        """
        Generate points for the specified explicit function.

        Parameters:
            num_points_per_range (int): Number of points per range.
            num_ranges (int): Number of ranges.

        Returns:
            np.ndarray: Array of generated points.
        """
        T = self.t_range[1] - self.t_range[0]
        T1 = T / num_ranges
        curve_points = []

        for i in range(num_ranges):
            t_values = np.linspace(i * T1, i * T1 + T1, num_points_per_range, endpoint=False)
            y_values, x_values = self.explicit_function(t_values)

            range_points = np.column_stack((x_values, -y_values))
            curve_points.append(range_points)

        curve_points = np.vstack(curve_points)
        curve_points = curve_points.astype(int) + np.array([250, 250])

        return curve_points
        
# Example usage:
if __name__ == "__main__":
    Bot1 = ImagePointGenerator("bot1_points_elephant.txt", 1)
    Bot2 = ImagePointGenerator("bot2_points_elephant.txt", 1)
    Bot3 = ImagePointGenerator("bot3_points_elephant.txt", 1)
    bot_points = [Bot1.generate_points(), Bot2.generate_points(), Bot3.generate_points()]
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
    converted_bot_points = []
    for bot_groups in bot_points:
        points = []
        for group in bot_groups:
            for point in group:
                points.append(point)
        
        converted_bot_points.append(np.array(points))

    visualizer = PointVisualizer(converted_bot_points, colors)
    visualizer.visualize_points()

    