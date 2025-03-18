import customtkinter
import tkinter as tk
from tkinter import filedialog
from tkintermapview import TkinterMapView
from PIL import Image, ImageTk
import geocoder
from . import image_processing as img_processing
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from custom_msg.msg import Coordinates

customtkinter.set_default_color_theme("blue")


class GuiNode(Node):
    def __init__(self):
        super().__init__('GUI_node')
        self.publisher = self.create_publisher(Coordinates, 'coordinates', 10)

    def publish_gps_data(self, x, y, toggle):
        msg = Coordinates()
        msg.x = float(x)
        msg.y = float(y)
        msg.toggle = int(toggle)
        self.publisher.publish(msg)


class RobotPainterGUI(customtkinter.CTk):
    APP_NAME = "Hue GUI"
    WIDTH = 1200
    HEIGHT = 600

    def __init__(self):
        super().__init__()
        self.title(RobotPainterGUI.APP_NAME)
        self.geometry(f"{RobotPainterGUI.WIDTH}x{RobotPainterGUI.HEIGHT}")
        self.minsize(RobotPainterGUI.WIDTH, RobotPainterGUI.HEIGHT)
        self.node = GuiNode()

        self.paint_image = None
        self.start_location_marker = None
        self.start_location = None  # Stores the chosen starting lat/lon
        self.cartesian_points = []  # Stores the (x, y) coordinates
        self.path_coordinates = []  # Stores the lat/lon coordinates
        self.marker_list = []

        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.frame_left = customtkinter.CTkFrame(master=self, width=300, corner_radius=0)
        self.frame_left.grid(row=0, column=0, padx=0, pady=0, sticky="nsew")

        self.frame_right = customtkinter.CTkFrame(master=self, corner_radius=0)
        self.frame_right.grid(row=0, column=1, pady=0, padx=0, sticky="nsew")

        self.load_coords_btn = customtkinter.CTkButton(master=self.frame_left, text="Load Coordinates", command=self.load_cartesian_coordinates)
        self.load_coords_btn.grid(row=0, column=0, padx=20, pady=(10, 10))

        self.process_image_btn = customtkinter.CTkButton(master=self.frame_left, text="Process Image", command=self.select_and_process_image)
        self.process_image_btn.grid(row=2, column=0, padx=20, pady=(10, 10))


        self.clear_path_btn = customtkinter.CTkButton(master=self.frame_left, text="Clear Path", command=self.clear_path)
        self.clear_path_btn.grid(row=1, column=0, padx=20, pady=(10, 10))

        # Tile Server Option Menu
        self.map_label = customtkinter.CTkLabel(self.frame_left, text="Tile Server:")
        self.map_label.grid(row=2, column=0, padx=20, pady=(10, 0))

        self.map_option_menu = customtkinter.CTkOptionMenu(
            self.frame_left, values=["OpenStreetMap", "Google normal", "Google satellite"], command=self.change_map)
        self.map_option_menu.grid(row=3, column=0, padx=20, pady=(10, 10))
        self.map_option_menu.set("OpenStreetMap")

        self.appearance_mode_label = customtkinter.CTkLabel(self.frame_left, text="Appearance Mode:")
        self.appearance_mode_label.grid(row=4, column=0, padx=20, pady=(10, 0))

        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            self.frame_left, values=["Light", "Dark"], command=self.change_appearance_mode)
        self.appearance_mode_optionemenu.grid(row=5, column=0, padx=20, pady=(10, 10))
        self.appearance_mode_optionemenu.set("Dark")

        self.map_widget = TkinterMapView(self.frame_right, corner_radius=0)
        self.map_widget.grid(row=0, column=0, sticky="nswe", padx=0, pady=0)
        self.frame_right.grid_rowconfigure(0, weight=1)
        self.frame_right.grid_columnconfigure(0, weight=1)

        self.lat_entry = customtkinter.CTkEntry(master=self.frame_left, placeholder_text="Start Latitude")
        self.lat_entry.grid(row=6, column=0, sticky="we", padx=20, pady=(12, 12))

        # Longitude Input
        self.lon_entry = customtkinter.CTkEntry(master=self.frame_left, placeholder_text="Start Longitude")
        self.lon_entry.grid(row=7, column=0, sticky="we", padx=20, pady=(12, 12))

        self.move_robot_btn = customtkinter.CTkButton(
            master=self.frame_left, text="Simulate Robot", command=self.simulate_robot_movement
        )
        self.move_robot_btn.grid(row=8, column=0, padx=20, pady=(10, 20))

        self.process_image_btn = customtkinter.CTkButton(master=self.frame_left, text="Process Image", command=self.select_and_process_image)
        self.process_image_btn.grid(row=2, column=0, padx=20, pady=(10, 10))
        
        self.send_gps_msg_btn = customtkinter.CTkButton(master=self.frame_left, text="Publish Waypoints", command=self.publish_gps_data)
        self.send_gps_msg_btn.grid(row=9, column=0, padx=20, pady=(10, 10))

        self.center_map_on_current_location()
        

        self.map_widget.canvas.bind("<Button-3>", self.on_right_click)
        self.map_widget.canvas.bind("<Control-Button-1>", self.on_right_click)
        self.map_widget.canvas.bind("<Button-2>", self.on_right_click)

        self.status_bar = customtkinter.CTkLabel(self, text="Status: Ready", height=24, anchor="w", fg_color="gray", text_color="white")
        self.status_bar.grid(row=2, column=0, columnspan=2, sticky="we")


    def set_status(self, message):
        self.status_bar.configure(text=message)

    def on_right_click(self, event):
        self.map_widget.delete_all_marker()
        lat, lon = self.map_widget.convert_canvas_coords_to_decimal_coords(event.x, event.y)
        self.lat_entry.delete(0, tk.END)
        self.lon_entry.delete(0, tk.END)
        self.lat_entry.insert(-1,lat)
        self.lon_entry.insert(-1,lon)
        self.map_widget.set_marker(
            lat,
            lon,
            text="Start",
            marker_color_circle="red",
            marker_color_outside="black",
        )


    def process_image(self, uploaded_image_path):
        k = 3
        border_size = 2
        min_points_per_edge = 50
        max_dist_betw_points = 5
        min_section_size = 10

        self.set_status("Status: Processing image...")


        waypoints_output_filename = 'image_waypoints.txt'
        img_rgb = img_processing.s0_prepare_img(uploaded_image_path, border_size=border_size, display=False)
        img_reduced_rgb = img_processing.s1_reduce_img_rgbs(img_rgb, k=k, display=False)

        self.set_status("Status: Generating waypoints...")

        img_edges = img_processing.s2_generate_edges(img_reduced_rgb, display=False)
        grouped_edges = img_processing.s3_group_edges(img_edges, edge_threshold=min_points_per_edge)
        ordered_edges = img_processing.s4_order_edges(grouped_edges, dist_thresh=max_dist_betw_points, section_size_thresh=min_section_size)
        simplified_paths = img_processing.s5_simplify_path(ordered_edges, epsilon=1.4)
        img_processing.s6_generate_output(simplified_paths, waypoints_output_filename)


    def center_map_on_current_location(self):
        try:
            g = geocoder.ip('me')
            if g.ok:
                lat, lon = g.latlng
                self.start_location = (lat, lon)
                self.map_widget.set_position(lat, lon)
            else:
                self.start_location = (37.7749, -122.4194)  # Default to San Francisco
                self.map_widget.set_position(*self.start_location)
            self.map_widget.set_zoom(15)
        except Exception as e:
            print(f"Error retrieving location: {e}")
            self.start_location = (37.7749, -122.4194)
            self.map_widget.set_position(*self.start_location)
            self.map_widget.set_zoom(15)

    def load_cartesian_coordinates(self):
        file_path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt"), ("CSV files", "*.csv")])
        if file_path:
            self.cartesian_points.clear()
            with open(file_path, "r") as file:
                for line in file:
                    x, y = map(float, line.strip().split(" "))
                    self.cartesian_points.append((x, y))
            self.plot_coordinates_on_map()

    def load_coordinates_triple(self,file_path):
        self.cartesian_points.clear()
        with open(file_path, "r") as file:
            for line in file:
                x, y, z = map(float, line.strip().split(", "))
                self.cartesian_points.append((x, y))
        self.plot_coordinates_on_map()

    def simulate_robot_movement(self):
        if not self.path_coordinates:
            print("No polyline to follow. Load or plot a path first!")
            return

        self.robot_marker = self.map_widget.set_marker(
            self.path_coordinates[0][0],
            self.path_coordinates[0][1],
            text="Robot",
            marker_color_circle="red",
            marker_color_outside="black",
        )
        self.move_robot_along_path(1)

    def move_robot_along_path(self, index):
        if index < len(self.path_coordinates):
            lat, lon = self.path_coordinates[index]
            self.robot_marker.set_position(lat, lon)
            self.after(100, self.move_robot_along_path, index + 1) 
        else:
            print("Robot has completed its journey along the path!")

    def plot_coordinates_on_map(self):
        if self.lat_entry.get( ) and self.lon_entry.get( ):
            start_lat = float(self.lat_entry.get() if self.lat_entry.get() else self.start_location)
            start_lon = float(self.lon_entry.get() if self.lon_entry.get() else self.start_location)
        else:
            start_lat, start_lon = self.start_location
        polyline_points = [] 

        for x, y in self.cartesian_points:
            lat = start_lat - (y * 0.0000015) 
            lon = start_lon + (x * 0.0000015)  
            self.path_coordinates.append((lat, lon))
            polyline_points.append((lat, lon))  

        if polyline_points:
            self.map_widget.set_path(polyline_points, color="blue", width=2)

    def clear_path(self):
        self.cartesian_points.clear()
        self.path_coordinates.clear()
        self.map_widget.delete_all_marker()
        self.map_widget.delete_all_path()

    def search_event(self, event=None):
        self.map_widget.set_address(self.entry.get())

    def change_appearance_mode(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_map(self, new_map: str):
        if new_map == "OpenStreetMap":
            self.map_widget.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")
        elif new_map == "Google normal":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        elif new_map == "Google satellite":
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)

    def select_and_process_image(self):
        """Open a file dialog to select an image and process it."""
        file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.jpg *.png *.jpeg")])
        if file_path:
            self.process_image(file_path)
            self.load_coordinates_triple("image_waypoints.txt")
            self.set_status("Status: Image processing complete")

    def publish_gps_data(self):
        coords = []
        file_path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt"), ("CSV files", "*.csv")])
        if file_path:
            with open(file_path, "r") as file:
                for line in file:
                    x, y, z = map(float, line.strip().split(", "))
                    coords.append((x, y, z))
        
            for x, y, toggle in coords:
                print(x,y,toggle)
                msg = Coordinates()
                msg.x = float(x)
                msg.y = float(y)
                msg.toggle = int(toggle)
                self.node.publish_gps_data(x, y, toggle)
                rclpy.spin_once(self.node, timeout_sec=0.1) 

    def start(self):
        self.mainloop()

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.after(100, self.ros_spin)  

    def on_closing(self):
        rclpy.shutdown()
        self.destroy()

def main(args=None):
    rclpy.init(args=args)
    app = RobotPainterGUI()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.start()

if __name__ == "__main__":
    main()
