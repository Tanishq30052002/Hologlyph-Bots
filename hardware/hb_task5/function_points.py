import numpy as np
import cv2

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

class FunctionPointVisualizer:
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
        max_points = len(self.bot_points[0])
        i = 0

        while i < max_points:
            image = np.zeros((self.video_size[1], self.video_size[0], 3), dtype=np.uint8)

            for bot_idx, bot_color in enumerate(self.colors):
                bot_points = self.bot_points[bot_idx]

                for k in range(min(i + 1, len(bot_points))):
                    cv2.circle(image, tuple(bot_points[k]), 5, bot_color, -1)

                    if k > 0:
                        cv2.line(image, tuple(bot_points[k-1]), tuple(bot_points[k]), bot_color, 2)

            cv2.imshow('Function Point Visualizer', image)

            key = cv2.waitKey(1)
            if key == 27:  # 'Esc' key
                break

            i += 1

        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Example usage:
if __name__ == "__main__":
    # Define an explicit function (e.g., a Lissajous curve)
    def lissajous_curve(t):
        """
        Lissajous curve function.

        Parameters:
            t (np.ndarray): Time values.

        Returns:
            tuple: Tuple of y and x values.
        """
        x = 200 * np.cos(t)
        y = 150 * np.sin(4 * t)
        return y, x

    # Create an instance of the FunctionPointGenerator
    lissajous_generator = FunctionPointGenerator(explicit_function=lissajous_curve, num_points=100, t_range=(0, 2*np.pi))

    num_points_per_range = 150
    num_ranges = 3

    # Generate points using the specified function
    points = lissajous_generator.generate_points(num_points_per_range, num_ranges)

    bot_points = [points[i*num_points_per_range:(i+1)*num_points_per_range] for i in range(num_ranges)]

    # Create an instance of the FunctionPointVisualizer
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # Red, Green, Blue
    visualizer = FunctionPointVisualizer(bot_points, colors)

    # Visualize the points in a video
    visualizer.visualize_points()
