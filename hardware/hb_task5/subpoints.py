import numpy as np
import cv2

class SubpointGenerator:
    """
    Class for generating subpoints along the sides of a polygon defined by its vertices.
    """
    def __init__(self, num_points_per_side=20):
        """
        Initialize the SubpointGenerator.

        Parameters:
            num_points_per_side (int): Number of subpoints to generate per side.
        """
        self.num_points_per_side = num_points_per_side

    def generate_subpoints(self, vertices):
        """
        Generate subpoints along the sides of the polygon.

        Parameters:
            vertices (list or np.ndarray): List or array of vertices.

        Returns:
            np.ndarray: Array of generated subpoints.
        """
        vertices_array = np.array(vertices)
        vertices_array = np.vstack([vertices_array, vertices_array[0]])

        edge_lengths = np.linalg.norm(np.diff(vertices_array, axis=0), axis=1)
        total_points = self.num_points_per_side * len(vertices)
        cumulative_lengths = np.cumsum(edge_lengths)
        normalized_lengths = cumulative_lengths / cumulative_lengths[-1]

        interpolated_points = []
        for i in range(len(vertices_array) - 1):
            side_points = np.linspace(0, 1, self.num_points_per_side, endpoint=False)
            side_interp = np.column_stack((np.interp(side_points, [0, 1], [vertices_array[i, 0], vertices_array[i + 1, 0]]),
                                           np.interp(side_points, [0, 1], [vertices_array[i, 1], vertices_array[i + 1, 1]])))
            interpolated_points.append(side_interp)

        return np.vstack(interpolated_points).astype(int)

class SubpointVisualizer:
    """
    Class for visualizing subpoints in a video using OpenCV.
    """
    def __init__(self, subpoints_list, colors, video_size=(500, 500)):
        """
        Initialize the SubpointVisualizer.

        Parameters:
            subpoints_list (list): List of arrays containing subpoints for each shape.
            colors (list): List of colors for each shape.
            video_size (tuple): Size of the video window (width, height).
        """
        self.subpoints_list = subpoints_list
        self.colors = colors
        self.video_size = video_size

    def visualize_subpoints(self):
        """
        Visualize the subpoints in a single video window using OpenCV.
        """
        max_points = max(len(subpoints) for subpoints in self.subpoints_list)
        i = 0

        while i < max_points:
            image = np.zeros((self.video_size[1], self.video_size[0], 3), dtype=np.uint8)

            for shape_idx, subpoints in enumerate(self.subpoints_list):
                shape_color = self.colors[shape_idx]

                for k in range(min(i + 1, len(subpoints))):
                    cv2.circle(image, tuple(subpoints[k]), 5, shape_color, -1)

                    if k > 0:
                        cv2.line(image, tuple(subpoints[k-1]), tuple(subpoints[k]), shape_color, 2)

            cv2.imshow('Subpoint Visualizer', image)

            key = cv2.waitKey(100)
            if key == 27:  # 'Esc' key
                break

            i += 1

        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Example usage:
if __name__ == "__main__":
    # Create an instance of SubpointGenerator
    subpoint_generator = SubpointGenerator(num_points_per_side=3)

    # Define vertices for each shape
    hexagon_vertices = np.array([[200, 150], [175, 200], [125, 200], [100, 150], [125, 100], [175, 100], [200, 150]])
    triangle_vertices = np.array([[300, 100], [400, 100], [350, 200], [300, 100]])
    rectangle_vertices = np.array([[200, 300], [400, 300], [400, 400], [200, 400], [200, 300]])

    # Generate subpoints for each shape
    hexagon_subpoints = subpoint_generator.generate_subpoints(vertices=hexagon_vertices)
    triangle_subpoints = subpoint_generator.generate_subpoints(vertices=triangle_vertices)
    rectangle_subpoints = subpoint_generator.generate_subpoints(vertices=rectangle_vertices)

    # Create an instance of SubpointVisualizer
    subpoints_list = [hexagon_subpoints, triangle_subpoints, rectangle_subpoints]
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # Red, Green, Blue
    visualizer = SubpointVisualizer(subpoints_list, colors)

    # Visualize the subpoints in a single video window
    visualizer.visualize_subpoints()
