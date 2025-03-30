import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, CubicSpline
from sklearn.cluster import KMeans
import random
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import matplotlib.transforms as transforms
from rdp import rdp
import copy
import sys
import os
from matplotlib.widgets import Button
from matplotlib.image import imread
sys.setrecursionlimit(20000)

# Input: Image path
# Output: RGB Image of shape (h, w, 3)

def s0_prepare_img(img_path, border_size = 2, display = False):
    img = cv2.imread(img_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Obtain image background color (assumed top-left pixel)
    background_color = img_rgb[0, 0].tolist()

    # Add a  border around the image
    img_with_border = cv2.copyMakeBorder(
        img_rgb,
        top = border_size,
        bottom = border_size,
        left = border_size,
        right = border_size,
        borderType = cv2.BORDER_CONSTANT,
        value = background_color)

    if display:
        plt.imshow(img_with_border)
        plt.axis('off')
        plt.show()

    return img_with_border

# Input: RGB image of shape (h, w, 3)
# Output: RGB image of shape (h, w, 3)

def s1_reduce_img_rgbs(img_rgb, k = 4, display = False):
    # Reshape input image for RGB processing
    img_height, img_width, _ = img_rgb.shape
    pixels = img_rgb.reshape(img_height * img_width, 3)

    # Obtain center RGB values of k clusters using k-means
    kmeans = KMeans(n_clusters = k, random_state = 0)
    kmeans.fit(pixels)
    selected_rgbs = np.round(kmeans.cluster_centers_).astype(int)

    # Initialize output image
    img_reduced_rgb = np.zeros_like(img_rgb)

    # Iterate through all image pixels and assign new output image pixel RGBs as the most similar K-means center
    for y in range(img_height):
        for x in range(img_width):
            rgb_errors = np.zeros((k))
            pixel_rgb = img_rgb[y, x, :]
            # Compute the error between the image pixel RGB and each k-means cluster center
            for i in range((k)):
                rgb_errors[i] = np.sqrt(np.mean((selected_rgbs[i] - pixel_rgb) ** 2))
            # Select RGB with lowest error and assign to the correpsonding output image pixel
            rgb_index = np.argmin(rgb_errors)
            img_reduced_rgb[y, x] = selected_rgbs[rgb_index]

    if display == True:
        plt.imshow([selected_rgbs])
        plt.show()
        plt.imshow(img_reduced_rgb)
        plt.show()

    return img_reduced_rgb

# Input: RGB image
# Output: Binary image of shape (h, w, 1)

def s2_generate_edges(img_rgb, display = False):
    # Initialize output edges image
    img_height, img_width, _ = img_rgb.shape
    img_edges = np.zeros_like(img_rgb[:, :, 0])

    # Establish pixel range for 'neighbor' processing
    y_range = [0, 1]
    x_range = [0, 1]

    # Iterate over all image pixels
    for y in range(img_height):
        for x in range(img_width):
            # Initialize a set to track all discovered RGBs among itself and its neighbors
            tracked_rgbs = set()
            # Iterate over relevant pixels (itself and its neighbors)
            for y_offset in y_range:
                for x_offset in x_range:
                    y_neighbor, x_neighbor = y + y_offset, x + x_offset
                    # Ensure pixel is within image dimesnion range
                    if 0 <= y_neighbor < img_height and 0 <= x_neighbor < img_width:
                        # Add discovered RGB to the tracked_rgbs set
                        neighbor_rgb = tuple(img_rgb[y_neighbor, x_neighbor])
                        tracked_rgbs.add(neighbor_rgb)
            # If more than one RGB is discovered within the pixel's range, set pixel as an edge
            if len(tracked_rgbs) > 1:
                img_edges[y, x] = 255

    if display == True:
        plt.imshow(img_edges)
        plt.show()

    return img_edges

# Input: Binary image
# Output: List of arrays of tuples (List length: number of edges; array length: number of pixels in given edge; tuples: pixel y and x coordinates)
def s3_group_edges(img_edges, edge_threshold = 50):
    # Initialize list of edges
    edges = []
    # Obtain connected compoennts
    num_labels, img_labels = cv2.connectedComponents(img_edges)
    for label in range(1, num_labels):
        edge_points = np.argwhere(img_labels == label)
        # Condition on minimum edge length
        if len(edge_points) >= edge_threshold:
            # Format and add edges to the output list
            edge_points_set = {tuple(point) for point in edge_points}
            edges.append(np.array(list(edge_points_set)))
    return edges

# Helper function: Identify the closest pixel to a given current pixel
def closest_point(curr, points):
    if len(points) == 0:
        raise ValueError('Invalid number of remaining points')
    min_dist = float('inf')
    min_point = None
    # Iterate through all remaining pixels
    for i, point in enumerate(points):
        # Calculate the distance between the current point and the remaining pixels
        dist = np.linalg.norm(np.array(curr) - np.array(point))
        # Update pixel if it has the shortest distance among traversed pixels
        if dist < min_dist:
            min_dist = dist
            min_point = point
    return min_dist, min_point

def helper(curr, remaining, all_sections, ordered_section, mode, dist_thresh,
           section_size_thresh):
    # Return if there are no longer any remaining pixels
    if len(remaining) == 0:
        all_sections.append(ordered_section)
        return

    # Obtain the closest remaining pixel to the current pixel
    min_dist, min_point = closest_point(curr, remaining)
    # The closest pixel is within the predetermined distance threhold
    if min_dist < dist_thresh:
        # Add pixel to end of ordered section when in forward mode
        if mode == 'forward':
            ordered_section.append(min_point)
        # Add pixel to start of ordered section when in backward mode
        else:
            ordered_section.insert(0, min_point)
        # Update remaining pixels and current pixel
        remaining.remove(min_point)
        curr = min_point
        # Call the function again
        helper(curr, remaining, all_sections, ordered_section, mode, dist_thresh,
               section_size_thresh)
    # The closest pixel is beyond the predetermined distance threshold
    else:
        # If in forward mode, update to backward mode, update current pixel and call the function again
        if mode == 'forward':
            mode = 'backward'
            curr = ordered_section[0]
            helper(curr, remaining, all_sections, ordered_section, mode, dist_thresh,
                   section_size_thresh)
        # If in backward mode and section is above minimum length, it is appended to the sections list and a new current pixel is identified
        elif mode == 'backward':
            if len(ordered_section) > section_size_thresh:
              all_sections.append(ordered_section)
            ordered_section = []
            curr = remaining.pop()
            ordered_section.append(curr)
            mode = 'forward'
            helper(curr, remaining, all_sections, ordered_section, mode, dist_thresh,
                   section_size_thresh)

# Input: List of arrays of tuples, int, int
# Output: List of arrays of tuples (List length: number of edges; array length: number of pixels in given edge; tuples: pixel y and x coordinates)
def s4_order_edges(edges, dist_thresh, section_size_thresh):
    ordered_edges = []
    for edge in edges:
        edge = list(map(tuple, edge))
        ordered_edge = []
        ordered_section = []
        remaining = set(edge)
        curr = remaining.pop()
        ordered_section.append(curr)
        mode = 'forward'
        all_sections = []

        helper(curr, remaining, all_sections, ordered_section, mode, dist_thresh,
               section_size_thresh)

        ordered_edges.append(all_sections)

    return ordered_edges

# List of arrays of tuples, int
# Output: List of arrays of tuples (List length: number of edges; array length: number of pixels in given edge; tuples: pixel y and x coordinates)
def s5_simplify_path(ordered_edges, epsilon):
    simplified_paths = []
    for edge in ordered_edges:
        edge = edge[0]
        simplified_edge = rdp(edge, epsilon=epsilon)
        simplified_paths.append(simplified_edge)
    return simplified_paths

# Input: List of arrays of tuples, string (output file path)
def s6_generate_output(simplified_paths, output_file):
  for i in range(len(simplified_paths)):
    if simplified_paths[i][-1] != simplified_paths[i][0]:
      simplified_paths[i].append(simplified_paths[i][0])
  #painting_toggle = 1
  all_waypoints = []
  for i, path in enumerate(simplified_paths):
    for j, coord in enumerate(path):
      x, y = coord
      if j == len(path) - 1:
        #painting_toggle = 1
        all_waypoints.append((x, y, 0))
      else:
        all_waypoints.append((x, y, 1))
      #painting_toggle = 0
    #painting_toggle = 1

  with open(output_file, "w") as file:
    for waypoint in all_waypoints:
      file.write(f"{waypoint[0]}, {waypoint[1]}, {waypoint[2]}\n")

  print('Generated Waypoints at ', output_file)

def s7_animate_output(paths, animation_output_filename):
  # First and second sets of corners
  corners1 = copy.deepcopy(paths[0])  # Replace with your first set
  corners2 = copy.deepcopy(paths[1])  # Replace with your second set

  # Function to generate equidistant points along the trajectory
  def generate_equidistant_points(points, num_points=100):
      # Compute distances between consecutive points
      distances = np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1))
      cumulative_distances = np.insert(np.cumsum(distances), 0, 0)  # Start with 0 distance

      # Create target distances ensuring all original points are included
      total_distance = cumulative_distances[-1]
      target_distances = np.linspace(0, total_distance, num=num_points)
      target_distances = np.unique(np.concatenate([target_distances, cumulative_distances]))
      target_distances.sort()  # Sort in ascending order

      # Interpolate x and y coordinates
      x = np.interp(target_distances, cumulative_distances, points[:, 0])
      y = np.interp(target_distances, cumulative_distances, points[:, 1])

      return np.column_stack((x, y))

  # Generate equidistant points for the two trajectories and transition
  pixels1 = generate_equidistant_points(corners1, num_points=200)
  pixels2 = generate_equidistant_points(corners2, num_points=200)
  transition = generate_equidistant_points(np.array([corners1[-1], corners2[0]]), num_points=10)

  # Combine all points
  pixels = np.vstack((pixels1, transition, pixels2))
  pixels = np.column_stack((pixels[:, 1], -pixels[:, 0])) # rotate to fix


  # Define frame ranges for each phase
  phase1_frames = len(pixels1)
  transition_frames = len(transition)
  phase2_frames = len(pixels2)

  # Load the robot PNG image
  robot_image = imread("/content/drive/Shareddrives/Senior Design :D/Code/hue_robot.jpeg")  # Replace with the path to your robot PNG

  # Set up the figure and axis
  fig, ax = plt.subplots(figsize=(10, 10))
  ax.set_xlim(np.min(pixels[:, 0]) - 25, np.max(pixels[:, 0]) + 25)
  ax.set_ylim(np.min(pixels[:, 1]) - 25, np.max(pixels[:, 1]) + 25)
  ax.set_title("Simulation of Hue Robot Painting Trajectory", fontsize=18)

  # Initialize line objects for each phase
  unpainted_line, = ax.plot(pixels[:, 0], pixels[:, 1], color='gray', lw=3, alpha=0.5, label="Planned Path")
  painted_phase1, = ax.plot([], [], c='blue', lw=3, label="Painted Path")
  painted_transition, = ax.plot([], [], c='red', lw=3, label="Unpainted Transition Path")
  painted_phase2, = ax.plot([], [], c='blue', lw=3)
  ax.set_xticks([])
  ax.set_xticks([], minor=True)
  ax.set_xticklabels([])
  ax.set_yticks([])
  ax.set_yticks([], minor=True)
  ax.set_yticklabels([])
  ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.02),
            fancybox=True, shadow=True, ncol=5, fontsize=15)

  # Create an imshow object for the robot image
  robot = ax.imshow(robot_image, extent=[0, 1, 0, 1])  # Placeholder extent; will be updated

  # Initialization function: plot background of each frame
  def init(): 
      painted_phase1.set_data([], [])
      painted_transition.set_data([], [])
      painted_phase2.set_data([], [])
      unpainted_line.set_data(pixels[:, 0], pixels[:, 1])  # Full unpainted line
      robot.set_extent([0, 1, 0, 1])  # Initialize robot position
      return painted_phase1, painted_transition, painted_phase2, unpainted_line, robot

  # Animation function: update robot position and painted line
  def animate(i):
      if i < phase1_frames:
          # First phase
          painted_phase1.set_data(pixels[:i + 1, 0], pixels[:i + 1, 1])
      elif i < phase1_frames + transition_frames:
          # Transition phase
          transition_start = phase1_frames
          painted_transition.set_data(pixels[transition_start:i + 1, 0], pixels[transition_start:i + 1, 1])
      else:
          # Second phase
          phase2_start = phase1_frames + transition_frames
          painted_phase2.set_data(pixels[phase2_start:i + 1, 0], pixels[phase2_start:i + 1, 1])

      # Update unpainted line
      unpainted_line.set_data(pixels[i:, 0], pixels[i:, 1])

      # Update robot position
      x, y = pixels[i]
      robot_size = 20  # Adjust the size of the robot image
      robot.set_extent([x - robot_size, x + robot_size, y - robot_size, y + robot_size])
      return painted_phase1, painted_transition, painted_phase2, unpainted_line, robot

  def on_key_press(event):
      if event.key == 'q':
          ani.event_source.stop()  # Stop the animation
          plt.close()  # Close the figure
  fig.canvas.mpl_connect('key_press_event', on_key_press)
  # Call the animator
  ani = FuncAnimation(fig, animate, init_func=init, frames=len(pixels), interval=50, blit=True)

  # Save the animation as a GIF
  ani.save('animation_final.gif', writer='pillow', fps=60, dpi=150)

  # Show the animation
  plt.show()

