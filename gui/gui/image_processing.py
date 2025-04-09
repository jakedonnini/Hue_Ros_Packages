import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, CubicSpline
from sklearn.cluster import KMeans
import random
from scipy.ndimage import median_filter
from PIL import Image
import math
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import matplotlib.transforms as transforms
from rdp import rdp
import copy
import sys
import os
from matplotlib.widgets import Button
from matplotlib.image import imread
from skimage import measure, morphology, color
from skimage.segmentation import find_boundaries
from skimage.util import img_as_ubyte
from shapely.geometry import Polygon, LineString

sys.setrecursionlimit(20000)

# Input: Image path
# Output: RGB Image of shape (h, w, 3)

def s0_prepare_img(img_path, border_size = 2, display = False):
    img = cv2.imread(img_path)
    #img = cv2.flip(img, 1)
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

# helper functions for s1_reduce_img_rgb

def reduce_img_with_primary_colors(img_rgb, primary_rgbs, display=False, apply_median_filter=True, filter_size=3):
    """
    Reduces the colors of an RGB image to the specified primary RGB colors and optionally applies median filtering to reduce noise.

    Parameters:
    - img_rgb (numpy.ndarray): Input RGB image of shape (h, w, 3).
    - primary_rgbs (list or numpy.ndarray): List or array of primary RGB colors.
                                              Shape should be (k, 3), where k is the number of colors.
    - display (bool): If True, displays the primary colors and the reduced image.
    - apply_median_filter (bool): If True, applies median filtering to the reduced image to reduce noise.
    - filter_size (int): Size of the median filter window (must be an odd integer).

    Returns:
    - numpy.ndarray: Reduced RGB image of shape (h, w, 3).
    """
    # Ensure primary_rgbs is a numpy array of integers
    primary_rgbs = np.array(primary_rgbs, dtype=int)
    k = primary_rgbs.shape[0]

    # Reshape input image for RGB processing
    img_height, img_width, _ = img_rgb.shape
    pixels = img_rgb.reshape(-1, 3)

    # Compute the Euclidean distance between each pixel and each primary RGB color
    # Using broadcasting for efficient computation
    distances = np.sqrt(np.mean((pixels[:, np.newaxis, :] - primary_rgbs[np.newaxis, :, :]) ** 2, axis=2))

    # Find the index of the closest primary RGB color for each pixel
    closest_color_indices = np.argmin(distances, axis=1)

    # Assign the closest primary RGB color to each pixel
    img_reduced_pixels = primary_rgbs[closest_color_indices]

    # Reshape back to the original image shape
    img_reduced_rgb = img_reduced_pixels.reshape(img_height, img_width, 3)

    if apply_median_filter:
        # Apply median filter to each channel separately
        img_reduced_rgb_filtered = np.zeros_like(img_reduced_rgb)
        for channel in range(3):
            img_reduced_rgb_filtered[:, :, channel] = median_filter(img_reduced_rgb[:, :, channel], size=filter_size)
        img_reduced_rgb = img_reduced_rgb_filtered

    if display:
        # Display the primary colors
        plt.figure(figsize=(8, 2))
        plt.imshow([primary_rgbs / 255.0])  # Normalize RGB values for display
        plt.axis('off')
        plt.title('Primary Colors')
        plt.show()

        # Display the reduced image
        plt.figure(figsize=(8, 8))
        plt.imshow(img_reduced_rgb.astype(np.uint8))
        plt.axis('off')
        plt.title('Image with Reduced Colors (Post-Processed)')
        plt.show()

    return img_reduced_rgb

def map_rgb_to_indices(img_rgb, primary_rgbs):
    """
    Maps each pixel's RGB value to its corresponding index in the primary_rgbs list.

    Parameters:
    - img_rgb (numpy.ndarray): RGB image of shape (h, w, 3).
    - primary_rgbs (list or numpy.ndarray): List or array of primary RGB colors.
                                           Shape should be (k, 3), where k is the number of colors.

    Returns:
    - numpy.ndarray: 2D array of shape (h, w) with color indices.
    """
    primary_rgbs = np.array(primary_rgbs, dtype=int)
    h, w, _ = img_rgb.shape
    img_indices = np.full((h, w), -1, dtype=int)  # Initialize with -1 for unmatched colors

    # Iterate over each primary color and assign index
    for idx, color_val in enumerate(primary_rgbs):
        matches = np.all(img_rgb == color_val, axis=2)
        img_indices[matches] = idx

    # Verify that all pixels have been matched
    if np.any(img_indices == -1):
        unmatched = np.sum(img_indices == -1)
        raise ValueError(f"There are {unmatched} pixels that do not match any primary colors.")

    return img_indices

def map_indices_to_rgb(img_indices, primary_rgbs):
    """
    Maps color indices back to their corresponding RGB values.

    Parameters:
    - img_indices (numpy.ndarray): 2D array of shape (h, w) with color indices.
    - primary_rgbs (list or numpy.ndarray): List or array of primary RGB colors.
                                           Shape should be (k, 3), where k is the number of colors.

    Returns:
    - numpy.ndarray: RGB image of shape (h, w, 3).
    """
    primary_rgbs = np.array(primary_rgbs, dtype=int)
    img_rgb = primary_rgbs[img_indices]
    return img_rgb

def get_neighbor_colors(img_indices, y, x, h, w):
    """
    Retrieves the color indices of the 8-connected neighbors of a pixel.

    Parameters:
    - img_indices (numpy.ndarray): 2D array of color indices.
    - y (int): Y-coordinate of the pixel.
    - x (int): X-coordinate of the pixel.
    - h (int): Height of the image.
    - w (int): Width of the image.

    Returns:
    - list: List of neighboring color indices.
    """
    neighbors = []
    for dy in [-1, 0, 1]:
        for dx in [-1, 0, 1]:
            if dy == 0 and dx == 0:
                continue  # Skip the center pixel
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w:
                neighbors.append(img_indices[ny, nx])
    return neighbors

def connected_component_analysis(img_reduced_rgb, primary_rgbs, min_size=50, display=False):
    """
    Post-processes a color-reduced RGB image to fix noise-induced color misassignments
    using Connected Component Analysis (CCA).

    This function identifies small connected regions of each primary color and reassigns
    them to the most common neighboring color, effectively removing isolated noise pixels.

    Parameters:
    - img_reduced_rgb (numpy.ndarray): Reduced RGB image of shape (h, w, 3).
    - primary_rgbs (list or numpy.ndarray): List or array of primary RGB colors.
                                           Shape should be (k, 3), where k is the number of colors.
    - min_size (int): Minimum size (in pixels) for a connected component to be retained.
                      Components smaller than this will be considered noise and removed.
    - display (bool): If True, displays the original and post-processed images.

    Returns:
    - numpy.ndarray: Post-processed RGB image of shape (h, w, 3).
    """
    # Step 1: Map RGB values to color indices
    img_indices = map_rgb_to_indices(img_reduced_rgb, primary_rgbs)
    h, w = img_indices.shape

    # Step 2: Initialize a copy for post-processing
    img_processed_indices = img_indices.copy()

    # Step 3: Perform CCA for each primary color
    for color_idx, color_val in enumerate(primary_rgbs):
        # Create a binary mask for the current color
        binary_mask = (img_indices == color_idx).astype(np.uint8)

        # Label connected components
        labeled_mask = measure.label(binary_mask, connectivity=2)  # 8-connectivity

        # Obtain properties of labeled regions
        props = measure.regionprops(labeled_mask)

        for prop in props:
            if prop.area < min_size:
                # This is a small region; reassign its pixels
                for coord in prop.coords:
                    y, x = coord
                    # Get neighboring color indices
                    neighbors = get_neighbor_colors(img_indices, y, x, h, w)
                    # Exclude the current color
                    neighbors = [n for n in neighbors if n != color_idx]
                    if neighbors:
                        # Assign the most common neighboring color
                        most_common = np.bincount(neighbors).argmax()
                        img_processed_indices[y, x] = most_common
                    else:
                        # If no neighbors are found (unlikely), keep the original color
                        pass

    # Step 4: Handle any pixels that might not match primary colors after reassignment
    # (This is a safety check; ideally, all pixels should match)
    if np.any(img_processed_indices == -1):
        unmatched = np.sum(img_processed_indices == -1)
        raise ValueError(f"There are {unmatched} pixels that do not match any primary colors after processing.")

    # Step 5: Map indices back to RGB
    img_post_processed_rgb = map_indices_to_rgb(img_processed_indices, primary_rgbs)

    if display:
        # Display the original and post-processed images side by side
        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.imshow(img_reduced_rgb.astype(np.uint8))
        plt.title('Original Reduced Image')
        plt.axis('off')

        plt.subplot(1, 2, 2)
        plt.imshow(img_post_processed_rgb.astype(np.uint8))
        plt.title(f'Post-Processed Image (min_size={min_size})')
        plt.axis('off')

        plt.show()

    return img_post_processed_rgb

def s1_reduce_img_rgbs(img_rgb, primary_rgbs, display=False, apply_median_filter=True, filter_size=3, min_size=50):
    img_reduced_rgb = reduce_img_with_primary_colors(img_rgb, primary_rgbs, display=display, apply_median_filter=apply_median_filter, filter_size=filter_size)
    img_post_processed_rgb = connected_component_analysis(img_reduced_rgb, primary_rgbs, min_size=min_size, display=False)
    return img_post_processed_rgb

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

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter
from PIL import Image

# Optimized Helper function: Identify the closest pixel to a given current pixel using vectorized operations
def closest_point_optim(curr, points_arr):
    """
    Find the closest point to 'curr' in 'points_arr' using vectorized distance computation.

    Parameters:
    - curr (tuple): Current point coordinates (y, x).
    - points_arr (numpy.ndarray): Array of points with shape (N, 2).

    Returns:
    - min_dist (float): Minimum Euclidean distance.
    - min_point (tuple): Coordinates of the closest point.
    """
    # Calculate differences
    diffs = points_arr - curr  # Shape: (N, 2)
    # Compute squared Euclidean distances
    dists = np.sqrt(np.sum(diffs ** 2, axis=1))  # Shape: (N,)
    # Find the index of the minimum distance
    min_idx = np.argmin(dists)
    min_dist = dists[min_idx]
    min_point = tuple(points_arr[min_idx])
    return min_dist, min_point

# Optimized Helper function: Iterative version to replace the recursive 'helper'
def helper_iterative_optim(curr, remaining_arr, all_sections, ordered_section, mode, dist_thresh, section_size_thresh):
    """
    Iteratively process points to form ordered sections based on distance thresholds.

    Parameters:
    - curr (tuple): Current point coordinates (y, x).
    - remaining_arr (numpy.ndarray): Array of remaining points with shape (N, 2).
    - all_sections (list): List to store all ordered sections.
    - ordered_section (list): Current ordered section.
    - mode (str): Current mode ('forward' or 'backward').
    - dist_thresh (float): Distance threshold to determine section continuation.
    - section_size_thresh (int): Minimum size threshold for a section to be retained.

    Returns:
    - None: Updates 'all_sections' and 'ordered_section' in place.
    """
    while True:
        if remaining_arr.size == 0:
            all_sections.append(ordered_section.copy())
            break

        # Find the closest point to the current point
        min_dist, min_point = closest_point_optim(np.array(curr), remaining_arr)

        if min_dist < dist_thresh:
            # Add point to the ordered_section based on the current mode
            if mode == 'forward':
                ordered_section.append(min_point)
            else:
                ordered_section.insert(0, min_point)

            # Remove the closest point from remaining_arr
            # Find the index of the min_point
            indices = np.where((remaining_arr == min_point).all(axis=1))[0]
            if len(indices) == 0:
                raise ValueError("Point to remove not found in remaining points.")
            remaining_arr = np.delete(remaining_arr, indices[0], axis=0)

            # Update current point
            curr = min_point
        else:
            if mode == 'forward':
                mode = 'backward'
                if len(ordered_section) == 0:
                    break  # No points to process
                curr = ordered_section[0]
            elif mode == 'backward':
                if len(ordered_section) > section_size_thresh:
                    all_sections.append(ordered_section.copy())
                ordered_section = []
                if remaining_arr.size == 0:
                    break
                # Select the last point as the new current point
                curr = tuple(remaining_arr[-1])
                ordered_section.append(curr)
                remaining_arr = remaining_arr[:-1]
                mode = 'forward'

    return remaining_arr, all_sections, ordered_section

# Optimized Main Function: Order edges
def s4_order_edges(edges, dist_thresh, section_size_thresh):
    """
    Orders edges by grouping connected pixels into sections based on distance thresholds.

    Parameters:
    - edges (list): List of arrays/lists containing pixel coordinates as (y, x) tuples.
    - dist_thresh (float): Distance threshold to determine if points are part of the same section.
    - section_size_thresh (int): Minimum number of pixels required for a section to be retained.

    Returns:
    - ordered_edges (list): List containing ordered sections for each edge.
    """
    ordered_edges = []
    for edge_idx, edge in enumerate(edges):
        # Convert edge to a NumPy array of shape (N, 2)
        edge_arr = np.array(edge, dtype=int)
        if edge_arr.size == 0:
            ordered_edges.append([])
            continue

        # Initialize ordered_section and all_sections
        ordered_section = []
        all_sections = []

        # Initialize remaining points as a NumPy array
        remaining_arr = edge_arr.copy()

        # Select the first point as the starting point
        curr = tuple(remaining_arr[0])
        ordered_section.append(curr)
        remaining_arr = remaining_arr[1:]

        # Initialize mode
        mode = 'forward'

        # Call the iterative helper
        remaining_arr, all_sections, ordered_section = helper_iterative_optim(
            curr, remaining_arr, all_sections, ordered_section, mode, dist_thresh, section_size_thresh
        )

        # Append any remaining ordered_section
        if len(ordered_section) > section_size_thresh:
            all_sections.append(ordered_section.copy())

        # Append to ordered_edges
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

def s6_helper_bounding_box(paths):
  copied_paths = copy.deepcopy(paths)
  min_x_vals = []
  min_y_vals = []
  for path in copied_paths:
    min_x_vals.append(np.min([x for x,y in path]))
    min_y_vals.append(np.min([y for x,y in path]))
  min_x = np.min(min_x_vals)
  min_y = np.min(min_y_vals)

  for path in copied_paths:
    for i in range(len(path)):
      path[i] = [int(path[i][0] - min_x), int(path[i][1] - min_y)]
    if path[-1] == path[0]:
      path.pop()
  return copied_paths

def convert_point(point):
  """Convert a point (which may be a tuple of np.int64s) to a list of two ints."""
  return [int(point[0]), int(point[1])]

def s6_helper_euclidean_distance(p, q):
  """Compute Euclidean distance between two points p and q."""
  return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

def s6_helper_rotate_path(path, idx):
  """
  Rotate the path so that the element at index `idx` becomes the first element.
  Then, append that element at the end to 'close the loop'.
  """
  rotated = path[idx:] + path[:idx]
  rotated.append(rotated[0])
  return rotated

def s65_generate_fill_paths(optimized_paths, dist_betw):
  """
  Given a closed polygon (first and last point are identical) defined by polygon_points,
  generate a continuous zigzag path that fills the polygon using horizontal scan lines.

  Parameters:
    optimized_paths
    dist_betw (float)

  Returns:
    list of tuple: Ordered waypoints representing the fill path.
  """
  fill_paths = []
  for shape in optimized_paths:
    shape_fill_path = []
    poly = Polygon(shape)
    minx, miny, maxx, maxy = poly.bounds

    row = 0  # used to alternate direction for continuity
    y = miny
    while y <= maxy:
        # Create an extended horizontal line at height y.
        scan_line = LineString([(minx - 10, y), (maxx + 10, y)])
        # Get the intersection between the polygon and the horizontal line.
        inter = poly.intersection(scan_line)

        if inter.is_empty:
            y += dist_betw
            continue

        # Handle cases where the intersection returns one or several segments.
        if inter.geom_type == "LineString":
            segments = [inter]
        elif inter.geom_type == "MultiLineString":
            segments = list(inter.geoms)  # use .geoms to iterate over individual segments
        else:
            y += dist_betw
            continue

        # Sort segments by their starting x-coordinate.
        segments = sorted(segments, key=lambda seg: seg.bounds[0])

        # For this scan line, collect points from each segment.
        row_points = []
        for seg in segments:
            xs, ys_seg = seg.xy
            seg_points = list(zip(xs, ys_seg))
            # Alternate direction every row for a continuous zigzag.
            if row % 2 == 1:
                seg_points.reverse()
            row_points.extend(seg_points)

        # Append the row's points to the overall path.
        shape_fill_path.extend(row_points)
        row += 1
        y += dist_betw

    fill_paths.append([list(t) for t in shape_fill_path])

  return fill_paths

def s6_optimize_waypoint_traversal(paths):
    """
    Given a list of disjoint paths (each path is a list of [x, y] waypoints),
    repeatedly choose the path that contains the waypoint closest to the current location,
    rotate that path so that the chosen waypoint is at both the start and end,
    update the current location, and remove the chosen path from further consideration.
    """
    bounded_paths = s6_helper_bounding_box(paths)
    current_location = [0, 0]
    ordered_paths = []
    remaining_paths = copy.deepcopy(bounded_paths)  # work on a copy so the original isn't modified

    while remaining_paths:
      best_distance = float('inf')
      best_path = None
      best_waypoint_idx = None
      best_path_index = None

      # Loop over each remaining path to find the closest waypoint
      for i, path in enumerate(remaining_paths):
        for j, waypoint in enumerate(path):
          wp = convert_point(waypoint)
          dist = s6_helper_euclidean_distance(current_location, waypoint)
          if dist < best_distance:
              best_distance = dist
              best_path = path
              best_waypoint_idx = j
              best_path_index = i

      # Rotate the chosen path so that the closest waypoint is first (and also last)
      new_path = s6_helper_rotate_path(best_path, best_waypoint_idx)
      ordered_paths.append(new_path)

      # Update the current location to the chosen waypoint
      current_location = new_path[0]

      # Remove the chosen path from the remaining paths
      del remaining_paths[best_path_index]

    return ordered_paths

# Input: List of arrays of tuples, string (output file path)
def s7_generate_output(simplified_paths, output_file):
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


# Input: List of arrays of tuples, string (output file path)
def s7_generate_output_with_filling(paths, theta, output_file, flip_y=False, flip_x=False, fill=False):
  if fill:
    outline_paths = paths[:len(paths) // 2]
    fill_paths = paths[len(paths) // 2:]
  else:
    outline_paths = paths
  theta = np.deg2rad(theta)
  rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
  if flip_y:
    rotation_matrix[1, 1] *= -1
  if flip_x:
    rotation_matrix[0, 0] *= -1
  new_paths = copy.deepcopy(outline_paths)
  for i in range(len(new_paths)):
    if new_paths[i][-1] != new_paths[i][0]:
      new_paths[i].append(new_paths[i][0])
  #painting_toggle = 1
  all_waypoints = []
  for i, path in enumerate(new_paths):
    for j, coord in enumerate(path):
      x, y = coord
      rotated_x, rotated_y = np.dot(rotation_matrix, np.array([x, y]))

      if j == 0:
        all_waypoints.append((-rotated_x, rotated_y, 0))
      else:
        all_waypoints.append((-rotated_x, rotated_y, 1))

      #all_waypoints.append((x, y, painting_toggle))
      #painting_toggle = 0
    #painting_toggle = 1

  if fill:
    all_fill_waypoints = []
    new_fill_paths = copy.deepcopy(fill_paths)
    for i, fill_path in enumerate(new_fill_paths):
        for j, coord in enumerate(fill_path):
          x, y = coord
          rotated_x, rotated_y = np.dot(rotation_matrix, np.array([x, y]))
          if j % 2 == 0:
            all_fill_waypoints.append((-rotated_x, rotated_y, 0))
          else:
            all_fill_waypoints.append((-rotated_x, rotated_y, 1))

    all_waypoints.extend(all_fill_waypoints)


  with open(output_file, "w") as file:
    for waypoint in all_waypoints:
      file.write(f"{waypoint[0]}, {waypoint[1]}, {waypoint[2]}\n")

  print('Generated Waypoints at ', output_file)