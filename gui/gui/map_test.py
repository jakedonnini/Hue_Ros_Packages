import tkinter as tk
from tkintermapview import TkinterMapView

root = tk.Tk()
map_widget = TkinterMapView(root, width=800, height=600)
map_widget.pack()
map_widget.set_tile_server("https://b.tile.openstreetmap.org/{z}/{x}/{y}.png")
map_widget.set_position(37.7749, -122.4194)  # San Francisco
map_widget.set_zoom(25)

root.mainloop()
