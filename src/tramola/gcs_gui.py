#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2 compatible GCS GUI for LoRa communication with offline map system

import Tkinter as tk
import ttk
import time
import threading
import math
import os
import urllib2
import base64
import io
import Queue  # Python 2's Queue

try:
    from PIL import Image, ImageTk

    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("PIL not available. Install with: pip install pillow")

from loralib import Lora, LoraGCSClient

# Create a global queue for tile download requests
tile_download_queue = Queue.Queue()


class TileDownloader(threading.Thread):
    """A thread that downloads map tiles in the background"""

    def __init__(self, callback=None):
        threading.Thread.__init__(self)
        self.daemon = True  # Thread will exit when main program exits
        self.running = True
        self.callback = callback
        self.last_callback_time = 0

    def run(self):
        while self.running:
            try:
                # Get a tile request from the queue with a timeout
                tile, url, filename = tile_download_queue.get(timeout=0.5)

                try:
                    # Download the tile
                    request = urllib2.Request(url)
                    request.add_header('User-Agent', 'BoatGCS/1.0')
                    response = urllib2.urlopen(request, timeout=5)
                    data = response.read()

                    # Save to cache
                    try:
                        with open(filename, 'wb') as f:
                            f.write(data)
                    except Exception as e:
                        # Silently ignore cache save errors
                        pass

                    # Load the image if PIL is available
                    if PIL_AVAILABLE:
                        tile.image = Image.open(io.BytesIO(data))
                        tile.photo_image = ImageTk.PhotoImage(tile.image)

                        # Call the callback if provided, but limit frequency to avoid flashing
                        current_time = time.time()
                        if self.callback and (
                                current_time - self.last_callback_time) > 0.25:  # Max 4 redraws per second
                            self.callback()
                            self.last_callback_time = current_time

                except Exception as e:
                    # Silently ignore download errors
                    pass

                # Mark this task as done
                tile_download_queue.task_done()

            except Queue.Empty:
                # No more tile requests, just wait
                time.sleep(0.1)

                # If we've downloaded tiles but not called the callback recently, do it now
                if self.callback and tile_download_queue.empty() and time.time() - self.last_callback_time > 0.5:
                    self.callback()
                    self.last_callback_time = time.time()

    def stop(self):
        self.running = False


class OfflineMapTile:
    """Represents a single map tile that can be cached for offline use"""

    def __init__(self, x, y, zoom, tile_server="https://tile.openstreetmap.org"):
        self.x = x
        self.y = y
        self.zoom = zoom
        self.tile_server = tile_server
        self.image = None
        self.photo_image = None
        self.is_loading = False

    def get_url(self):
        """Get the URL for this tile"""
        return "{server}/{zoom}/{x}/{y}.png".format(
            server=self.tile_server,
            zoom=self.zoom,
            x=self.x,
            y=self.y
        )

    def get_filename(self, cache_dir="map_cache"):
        """Get the filename for the cached tile"""
        if not os.path.exists(cache_dir):
            os.makedirs(cache_dir)
        return os.path.join(cache_dir, "tile_{zoom}_{x}_{y}.png".format(
            zoom=self.zoom,
            x=self.x,
            y=self.y
        ))

    def load(self, callback=None):
        """Load this tile from cache or queue for download"""
        if not PIL_AVAILABLE:
            return False

        # Check cache first
        filename = self.get_filename()
        if os.path.exists(filename):
            try:
                self.image = Image.open(filename)
                self.photo_image = ImageTk.PhotoImage(self.image)
                return True
            except Exception as e:
                # Silently ignore cache save errors
                pass

        # Queue for download if not already loading
        if not self.is_loading:
            self.is_loading = True
            url = self.get_url()
            # Add to download queue
            tile_download_queue.put((self, url, filename))

        return False


class SimpleMapCanvas(tk.Canvas):
    """A simple map representation showing boat position and waypoints with offline map support"""

    def __init__(self, parent, **kwargs):
        tk.Canvas.__init__(self, parent, **kwargs)
        self.config(bg='lightblue')
        self.waypoints = []
        self.boat_position = None
        self.boat_heading = 0
        self.center_lat = 39.9334  # Ankara latitude
        self.center_lon = 32.8597  # Ankara longitude
        self.zoom = 13  # Map zoom level (OpenStreetMap zoom)
        self.tile_size = 256  # Standard OSM tile size
        self.initialized = False
        self.map_tiles = {}  # Cache of loaded map tiles
        self.show_map = PIL_AVAILABLE  # Only show map if PIL is available

        # For scheduled redraws
        self._redraw_after_id = None

        # Create a label for status messages
        self.status_text = tk.StringVar(value="Map Ready")

        # Start the tile downloader thread
        self.tile_downloader = TileDownloader(callback=self.on_tile_downloaded)
        self.tile_downloader.start()

        # Bind to configure event to handle resize
        self.bind("<Configure>", self.on_resize)

        # Bind mouse events for dragging
        self.bind("<ButtonPress-1>", self.clicked_position_and_on_mouse_down_method)
        self.bind("<B1-Motion>", self.on_mouse_drag)

        # Bind mouse wheel for zooming
        self.bind("<MouseWheel>", self.on_mouse_wheel)  # Windows
        self.bind("<Button-4>", self.on_mouse_wheel)  # Linux scroll up
        self.bind("<Button-5>", self.on_mouse_wheel)  # Linux scroll down
        self.bind("<ButtonRelease-1>", self.on_mouse_release)

        self.drag_offset_x = 0
        self.drag_offset_y = 0
        self.is_dragging = False


    def clicked_position_and_on_mouse_down_method(self, event):

        """
        on_mouse_down_methodu
        """
        self.last_x = event.x
        self.last_y = event.y
        self.is_dragging = True

        """
        clicked_position_methodu
        """
        x = event.x
        y = event.y

        lat, lon = self.pixel_to_lat_lon(x, y)

        if hasattr(app, 'lat_entry_var') and hasattr(app, 'lon_entry_var'):
            app.lat_entry_var.set("{:.6f}".format(lat))
            app.lon_entry_var.set("{:.6f}".format(lon))


    def pixel_to_lat_lon(self, pixelx, pixely):

        width = self.winfo_width() or 600
        height = self.winfo_height() or 500

        center_screen_x = width / 2
        center_screen_y = height / 2

        center_x = (self.center_lon + 180) / 360.0 * self.mercator_factor()
        center_lat_rad = math.radians(self.center_lat)
        center_y = (1 - math.log(
            math.tan(center_lat_rad) + 1 / math.cos(center_lat_rad)) / math.pi) / 2 * self.mercator_factor()

        lon = (center_x + pixelx - center_screen_x -self.drag_offset_x) / self.mercator_factor() * 360 - 180
        lat = math.degrees(math.atan(math.sinh(math.pi - 2.0 * math.pi * (center_y + pixely - center_screen_y - self.drag_offset_y) /
                                               self.mercator_factor())))

        return lat, lon


    def on_tile_downloaded(self):
        """Called when a tile has been downloaded"""
        self.schedule_redraw()

    def schedule_redraw(self):
        """Schedule a redraw with a debounce to avoid flashing"""
        if self._redraw_after_id:
            self.after_cancel(self._redraw_after_id)
        self._redraw_after_id = self.after(100, self.redraw)

    def on_mouse_down(self, event):
        """Handle mouse button press"""
        self.last_x = event.x
        self.last_y = event.y
        self.is_dragging = True

    def on_mouse_drag(self, event):
        if not self.initialized:
            return

        dx = event.x - self.last_x
        dy = event.y - self.last_y
        self.last_x = event.x
        self.last_y = event.y

        self.drag_offset_x += dx
        self.drag_offset_y += dy

        self.redraw()

    def on_mouse_release(self, event):
        if not self.initialized or not self.is_dragging:
            return
        self.is_dragging = False

    def on_mouse_wheel(self, event):
        """Handle mouse wheel for zooming"""
        if not self.initialized:
            return

        # Determine zoom direction based on the event
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            # Zoom in
            self.set_zoom(self.zoom + 1)
        elif event.num == 5 or (hasattr(event, 'delta') and event.delta < 0):
            # Zoom out
            self.set_zoom(self.zoom - 1)

    def on_resize(self, event):
        """Handle resize events"""
        if not self.initialized and event.width > 1 and event.height > 1:
            self.initialized = True
        self.schedule_redraw()

    def set_center(self, lat, lon):
        """Set the center point of the map"""
        self.center_lat = lat
        self.center_lon = lon
        self.schedule_redraw()

    def set_zoom(self, zoom):
        """Set the zoom level"""
        # Ensure zoom is in valid range (0-19 for most tile servers)
        zoom = max(1, min(19, zoom))
        if zoom != self.zoom:
            self.zoom = zoom
            self.status_text.set("Map Zoom: {}".format(zoom))
            self.schedule_redraw()

    def mercator_factor(self):
        """Get the factor for Mercator projection at current zoom"""
        return self.tile_size * (2 ** self.zoom)

    def lat_lon_to_pixel(self, lat, lon):
        """Convert latitude and longitude to pixel coordinates"""
        # Get map dimensions
        width = self.winfo_width() or 600
        height = self.winfo_height() or 500

        # Calculate the world coordinates
        x = (lon + 180) / 360.0 * self.mercator_factor()
        lat_rad = math.radians(lat)
        y = (1 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2 * self.mercator_factor()

        # Calculate the center of the map in world coordinates
        center_x = (self.center_lon + 180) / 360.0 * self.mercator_factor()
        center_lat_rad = math.radians(self.center_lat)
        center_y = (1 - math.log(
            math.tan(center_lat_rad) + 1 / math.cos(center_lat_rad)) / math.pi) / 2 * self.mercator_factor()

        # Calculate the pixel coordinates relative to the center
        pixel_x = width / 2 + (x - center_x)
        pixel_y = height / 2 + (y - center_y)

        pixel_x += self.drag_offset_x
        pixel_y += self.drag_offset_y

        return pixel_x, pixel_y

    def get_tile_bounds(self):
        """Get the bounds of visible tiles"""
        if not self.initialized:
            return (0, 0, 0, 0)

        # Get canvas dimensions
        width = self.winfo_width() or 600
        height = self.winfo_height() or 500

        # Calculate the world coordinates of the center
        center_x = (self.center_lon + 180) / 360.0 * self.mercator_factor()
        center_lat_rad = math.radians(self.center_lat)
        center_y = (1 - math.log(
            math.tan(center_lat_rad) + 1 / math.cos(center_lat_rad)) / math.pi) / 2 * self.mercator_factor()

        # Drag offset (in pixel) is subtracted to simulate panning
        center_x -= self.drag_offset_x
        center_y -= self.drag_offset_y

        # Calculate the tile coordinates of the center
        center_tile_x = int(center_x / self.tile_size)
        center_tile_y = int(center_y / self.tile_size)

        # Calculate the tiles needed to fill the view
        tiles_x = int(width / self.tile_size) + 2  # Add margin
        tiles_y = int(height / self.tile_size) + 2  # Add margin

        # Calculate the bounds
        min_tile_x = center_tile_x - tiles_x // 2
        max_tile_x = center_tile_x + tiles_x // 2
        min_tile_y = center_tile_y - tiles_y // 2
        max_tile_y = center_tile_y + tiles_y // 2

        return (min_tile_x, min_tile_y, max_tile_x, max_tile_y)

    def pixel_to_tile_position(self, tile_x, tile_y):
        """Convert tile coordinates to pixel positions on the canvas"""
        # Get canvas dimensions
        width = self.winfo_width() or 600
        height = self.winfo_height() or 500

        # Calculate the world coordinates of the center
        center_x = (self.center_lon + 180) / 360.0 * self.mercator_factor()
        center_lat_rad = math.radians(self.center_lat)
        center_y = (1 - math.log(
            math.tan(center_lat_rad) + 1 / math.cos(center_lat_rad)) / math.pi) / 2 * self.mercator_factor()

        # Calculate the tile coordinates of the center
        center_tile_x = center_x / self.tile_size
        center_tile_y = center_y / self.tile_size

        # Calculate the pixel offset from the center
        pixel_x = width / 2 + (tile_x - center_tile_x) * self.tile_size + self.drag_offset_x
        pixel_y = height / 2 + (tile_y - center_tile_y) * self.tile_size + self.drag_offset_y

        return pixel_x, pixel_y

    def set_boat_position(self, lat, lon, heading=0):
        """Update the boat's position and heading"""
        self.boat_position = (lat, lon)
        self.boat_heading = heading

        # If no center has been set yet, center on the boat
        if self.center_lat == 0 and self.center_lon == 0:
            self.center_lat = lat
            self.center_lon = lon

        self.schedule_redraw()

    def add_waypoint(self, lat, lon):
        """Add a waypoint to the map"""
        self.waypoints.append((lat, lon))

        # If this is the first waypoint and no center has been set,
        # center the map on this waypoint
        if len(self.waypoints) == 1 and self.center_lat == 0 and self.center_lon == 0:
            self.center_lat = lat
            self.center_lon = lon

        self.schedule_redraw()

    def redraw(self):
        """Redraw the entire map"""
        # Skip redraw if canvas is not fully initialized
        if not self.winfo_width() or not self.winfo_height():
            return

        self.delete("all")  # Remove all items

        # Draw the map tiles if enabled
        if self.show_map:
            self.draw_map_tiles()

        # Draw waypoints
        for i, (lat, lon) in enumerate(self.waypoints):
            x, y = self.lat_lon_to_pixel(lat, lon)
            self.create_oval(x - 5, y - 5, x + 5, y + 5, fill="green", outline="black", tags="waypoint")
            self.create_text(x, y - 15, text="WP" + str(i + 1), fill="black", tags="waypoint")

            # Draw line connecting waypoints
            if i > 0:
                prev_x, prev_y = self.lat_lon_to_pixel(self.waypoints[i - 1][0], self.waypoints[i - 1][1])
                self.create_line(prev_x, prev_y, x, y, fill="green", dash=(4, 4), tags="waypoint")

        # Draw boat position and heading indicator
        if self.boat_position:
            x, y = self.lat_lon_to_pixel(self.boat_position[0], self.boat_position[1])
            self.create_oval(x - 8, y - 8, x + 8, y + 8, fill="red", outline="black", tags="boat")

            # Draw heading indicator
            heading_rad = math.radians(self.boat_heading)
            hx = x + 20 * math.sin(heading_rad)
            hy = y - 20 * math.cos(heading_rad)
            self.create_line(x, y, hx, hy, arrow=tk.LAST, width=2, fill="red", tags="boat")

            # Draw coordinates text
            pos_text = "Lat: {:.6f}, Lon: {:.6f}".format(self.boat_position[0], self.boat_position[1])
            self.create_text(x, y + 20, text=pos_text, fill="black", tags="boat")

        # Draw center coordinates and zoom level
        center_info = "Center: {:.6f}, {:.6f} (Zoom: {})".format(
            self.center_lat, self.center_lon, self.zoom)
        self.create_text(10, self.winfo_height() - 10, text=center_info,
                         anchor=tk.SW, fill="black", tags="info")

        # Draw instructions
        instructions = "Drag to pan, Scroll to zoom"
        self.create_text(self.winfo_width() - 10, self.winfo_height() - 10,
                         text=instructions, anchor=tk.SE, fill="black", tags="info")

        # Reset the redraw after ID
        self._redraw_after_id = None

    def draw_map_tiles(self):
        """Draw the map tiles"""
        # Get the bounds of visible tiles
        min_tile_x, min_tile_y, max_tile_x, max_tile_y = self.get_tile_bounds()

        # Get the maximum number of tiles at this zoom level
        max_tiles = 2 ** self.zoom

        # Ensure tile coordinates are within bounds
        min_tile_x = max(0, min(max_tiles - 1, min_tile_x))
        max_tile_x = max(0, min(max_tiles - 1, max_tile_x))
        min_tile_y = max(0, min(max_tiles - 1, min_tile_y))
        max_tile_y = max(0, min(max_tiles - 1, max_tile_y))

        # Count of tiles needing to be downloaded
        tiles_to_download = 0

        # Loop through and draw visible tiles
        for tile_x in range(min_tile_x, max_tile_x + 1):
            for tile_y in range(min_tile_y, max_tile_y + 1):
                # Get or create the tile
                tile_key = "{}_{}_{}_".format(self.zoom, tile_x, tile_y)

                if tile_key not in self.map_tiles:
                    # Create a new tile
                    tile = OfflineMapTile(tile_x, tile_y, self.zoom)
                    if tile.load(callback=self.on_tile_downloaded):
                        self.map_tiles[tile_key] = tile
                    else:
                        self.map_tiles[tile_key] = tile
                        tiles_to_download += 1

                # Draw the tile if it's loaded
                if tile_key in self.map_tiles:
                    tile = self.map_tiles[tile_key]
                    if tile.photo_image:
                        # Calculate the pixel position for this tile
                        pixel_x, pixel_y = self.pixel_to_tile_position(tile_x, tile_y)
                        self.create_image(
                            pixel_x, pixel_y,
                            image=tile.photo_image,
                            anchor=tk.NW,
                            tags="tile"
                        )
                    else:
                        # Draw placeholder for tiles being downloaded
                        pixel_x, pixel_y = self.pixel_to_tile_position(tile_x, tile_y)
                        self.create_rectangle(
                            pixel_x, pixel_y,
                            pixel_x + self.tile_size, pixel_y + self.tile_size,
                            fill="lightgray", outline="gray",
                            tags="tile_placeholder"
                        )

        # Update status if tiles are being downloaded
        if tiles_to_download > 0:
            self.status_text.set("Downloading {} tiles...".format(tiles_to_download))

    def cleanup(self):
        """Clean up resources, stop threads"""
        if hasattr(self, 'tile_downloader'):
            self.tile_downloader.stop()


class GCSApp(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.title("Boat Ground Control Station")
        self.geometry("1280x720")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Initialize LoRa connection (default values)
        self.lora = None
        self.gcs_client = None
        self.connected = False
        self.update_thread = None
        self.running = False
        self.waypoints = []

        self.create_widgets()
        self.update_connection_status()

        # Log that we're starting with Ankara as the map center
        self.log_message("Map initialized to Ankara coordinates")

    def create_widgets(self):
        # Create main frames
        left_frame = ttk.Frame(self)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        right_frame = ttk.Frame(self)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=10, pady=10)

        # Map frame
        map_frame = ttk.LabelFrame(left_frame, text="Navigation Map")
        map_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.map_canvas = SimpleMapCanvas(map_frame, width=600, height=500)
        self.map_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.lat_entry_var = tk.StringVar()
        self.lon_entry_var = tk.StringVar()

        # Map status bar
        map_status_frame = ttk.Frame(left_frame)
        map_status_frame.pack(fill=tk.X, pady=2)

        ttk.Label(map_status_frame, textvariable=self.map_canvas.status_text).pack(side=tk.LEFT)

        # Connection frame
        connection_frame = ttk.LabelFrame(right_frame, text="Connection")
        connection_frame.pack(fill=tk.X, pady=5)

        ttk.Label(connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_entry = ttk.Entry(connection_frame, width=15)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        self.port_entry.insert(0, "/dev/ttyACM0")

        ttk.Label(connection_frame, text="Baud:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_entry = ttk.Entry(connection_frame, width=10)
        self.baud_entry.grid(row=0, column=3, padx=5, pady=5)
        self.baud_entry.insert(0, "9600")

        self.connect_btn = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5, pady=5)

        self.status_label = ttk.Label(connection_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=5, padx=5, pady=5)

        # Boat status frame
        status_frame = ttk.LabelFrame(right_frame, text="Boat Status")
        status_frame.pack(fill=tk.X, pady=5)

        # Add position to status labels
        status_labels = ["Latitude", "Longitude", "Speed (Real)", "Speed (Req)",
                         "Heading", "Yaw Requested", "Thruster Left", "Thruster Right"]
        self.status_values = {}

        for i, label in enumerate(status_labels):
            ttk.Label(status_frame, text=label + ":").grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)
            value_var = tk.StringVar(value="N/A")
            self.status_values[label] = value_var
            ttk.Label(status_frame, textvariable=value_var).grid(row=i, column=1, padx=5, pady=2, sticky=tk.W)

        # Waypoint control frame
        waypoint_frame = ttk.LabelFrame(right_frame, text="Waypoint Control")
        waypoint_frame.pack(fill=tk.X, pady=5)

        ttk.Label(waypoint_frame, text="Latitude:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.lat_entry = ttk.Entry(waypoint_frame, width=15, textvariable=self.lat_entry_var)
        self.lat_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(waypoint_frame, text="Longitude:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.lon_entry = ttk.Entry(waypoint_frame, width=15, textvariable=self.lon_entry_var)
        self.lon_entry.grid(row=1, column=1, padx=5, pady=5)

        waypoint_btn_frame = ttk.Frame(waypoint_frame)
        waypoint_btn_frame.grid(row=2, column=0, columnspan=1, pady=5)

        self.add_wp_btn = ttk.Button(waypoint_btn_frame, text="Add Waypoint", command=self.add_waypoint)
        self.add_wp_btn.pack(side=tk.LEFT, padx=5)

        # Mission control frame
        mission_frame = ttk.LabelFrame(right_frame, text="Mission Control")
        mission_frame.pack(fill=tk.X, pady=5)

        self.start_mission_btn = ttk.Button(mission_frame, text="Start Mission", command=self.start_mission)
        self.start_mission_btn.pack(fill=tk.X, padx=5, pady=5)

        self.emergency_btn = ttk.Button(mission_frame, text="EMERGENCY STOP",
                                        command=self.emergency_stop, style="Emergency.TButton")
        self.emergency_btn.pack(fill=tk.X, padx=5, pady=5)

        # Configure emergency button style
        self.style = ttk.Style()
        self.style.configure("Emergency.TButton", foreground="white", background="red", font=("Helvetica", 12, "bold"))

        # Map controls
        map_control_frame = ttk.LabelFrame(right_frame, text="Map Controls")
        map_control_frame.pack(fill=tk.X, pady=5)

        # Zoom controls
        zoom_frame = ttk.Frame(map_control_frame)
        zoom_frame.pack(fill=tk.X, pady=5)

        ttk.Label(zoom_frame, text="Zoom:").pack(side=tk.LEFT, padx=5)
        self.zoom_scale = ttk.Scale(zoom_frame, from_=1, to=19, orient=tk.HORIZONTAL,
                                    command=self.update_zoom)
        self.zoom_scale.set(13)  # Default zoom level for OSM
        self.zoom_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Center on boat button
        self.center_boat_btn = ttk.Button(map_control_frame, text="Center on Boat",
                                          command=self.center_on_boat)
        self.center_boat_btn.pack(fill=tk.X, padx=5, pady=5)

        # Refresh map tiles button
        self.refresh_map_btn = ttk.Button(map_control_frame, text="Refresh Map Tiles",
                                          command=self.refresh_map_tiles)
        self.refresh_map_btn.pack(fill=tk.X, padx=5, pady=5)

        # Toggle map display
        self.show_map_var = tk.BooleanVar(value=PIL_AVAILABLE)
        self.show_map_cb = ttk.Checkbutton(map_control_frame, text="Show Map Tiles",
                                           variable=self.show_map_var,
                                           command=self.toggle_map_display)
        self.show_map_cb.pack(fill=tk.X, padx=5, pady=5)

        # Status console
        console_frame = ttk.LabelFrame(left_frame, text="Status Console")
        console_frame.pack(fill=tk.X, pady=5)

        self.console = tk.Text(console_frame, height=6, width=40, font=("Courier", 9))
        self.console.pack(fill=tk.BOTH, padx=5, pady=5)
        self.console.config(state=tk.DISABLED)

        # Add initial status message
        if PIL_AVAILABLE:
            self.log_message("Map system ready. Drag to pan, scroll to zoom.")
        else:
            self.log_message("WARNING: PIL/Pillow not available. Offline maps disabled.")
            self.log_message("Install with: pip install pillow")

    def refresh_map_tiles(self):
        """Force a refresh of map tiles"""
        # Clear the tile cache
        self.map_canvas.map_tiles = {}
        self.map_canvas.schedule_redraw()
        self.log_message("Map tiles refreshed")

    def toggle_map_display(self):
        """Toggle the display of map tiles"""
        self.map_canvas.show_map = self.show_map_var.get()
        self.map_canvas.schedule_redraw()

    def center_on_boat(self):
        """Center the map on the boat's position"""
        if self.map_canvas.boat_position:
            lat, lon = self.map_canvas.boat_position
            self.map_canvas.set_center(lat, lon)
            self.log_message("Map centered on boat position")
        else:
            self.log_message("No boat position available")

    def toggle_connection(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        try:
            port = self.port_entry.get()
            baud = int(self.baud_entry.get())

            self.log_message("Connecting to {} at {} baud...".format(port, baud))

            self.lora = Lora(port=port, baud_rate=baud)
            self.gcs_client = LoraGCSClient(self.lora)

            # Start data updates
            self.running = True
            self.update_thread = threading.Thread(target=self.update_loop)
            self.update_thread.daemon = True
            self.update_thread.start()

            self.gcs_client.start_data_requests()
            self.connected = True
            self.log_message("Connected successfully")
            self.update_connection_status()

        except Exception as e:
            self.log_message("Connection error: {}".format(e))
            if self.lora:
                self.lora.close()
                self.lora = None
                self.gcs_client = None

    def disconnect(self):
        self.log_message("Disconnecting...")

        # First, stop the GUI update loop
        self.running = False

        # Stop the GCS client data requests first
        if self.gcs_client:
            self.gcs_client.close()
            self.gcs_client = None

        # Wait for the GUI update thread to finish
        if self.update_thread:
            self.update_thread.join(timeout=2.0)
            self.update_thread = None

        # Finally, close the LoRa connection
        if self.lora:
            self.lora.close()
            self.lora = None

        self.connected = False
        self.update_connection_status()
        self.log_message("Disconnected")

    def update_connection_status(self):
        if self.connected:
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.start_mission_btn.state(["!disabled"])
            self.emergency_btn.state(["!disabled"])
            self.add_wp_btn.state(["!disabled"])
        else:
            self.status_label.config(text="Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
            self.start_mission_btn.state(["disabled"])
            self.emergency_btn.state(["disabled"])
            self.add_wp_btn.state(["disabled"])

            # Reset status values
            for var in self.status_values.values():
                var.set("N/A")

    def update_loop(self):
        """Thread to update GUI with latest data from the LoRa client"""
        while self.running and self.gcs_client:
            try:
                # Update boat status indicators
                if self.gcs_client.location:
                    lat, lon = self.gcs_client.location
                    self.map_canvas.set_boat_position(lat, lon, self.gcs_client.heading)

                    # Update position in status tab
                    self.status_values["Latitude"].set("{:.6f}°".format(lat))
                    self.status_values["Longitude"].set("{:.6f}°".format(lon))

                # Update status values
                self.status_values["Speed (Real)"].set("{:.2f} m/s".format(self.gcs_client.speed_real))
                self.status_values["Speed (Req)"].set("{:.2f} m/s".format(self.gcs_client.speed_requested))
                self.status_values["Heading"].set("{:.1f}°".format(self.gcs_client.heading))
                self.status_values["Yaw Requested"].set("{:.1f}°".format(self.gcs_client.yaw_requested))

                # Handle thruster values as tuple (left, right)
                if isinstance(self.gcs_client.thruster_requested, tuple) and len(
                        self.gcs_client.thruster_requested) >= 2:
                    self.status_values["Thruster Left"].set("{:.1f}%".format(self.gcs_client.thruster_requested[0]))
                    self.status_values["Thruster Right"].set("{:.1f}%".format(self.gcs_client.thruster_requested[1]))
                else:
                    self.status_values["Thruster Left"].set("N/A")
                    self.status_values["Thruster Right"].set("N/A")

            except Exception as e:
                self.log_message("Update error: {}".format(e))

            time.sleep(0.1)

    def log_message(self, message):
        """Add a message to the console"""
        self.console.config(state=tk.NORMAL)
        self.console.insert(tk.END, time.strftime("%H:%M:%S") + " - " + message + "\n")
        self.console.see(tk.END)  # Scroll to the bottom
        self.console.config(state=tk.DISABLED)

    def add_waypoint(self):
        """Add a waypoint to the map and send to the boat"""
        # If connected, send to boat
        if not (self.connected and self.gcs_client):
            self.log_message("Failed to send waypoint to boat")
            return
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())

            response = self.gcs_client.add_waypoint(lat, lon)
            if response == "OK":
                self.log_message("Waypoint added at {:.6f}, {:.6f}".format(lat, lon))
                # Add to local waypoint list
                self.waypoints.append((lat, lon))
                self.map_canvas.add_waypoint(lat, lon)
            else:
                self.log_message("Failed to send waypoint to boat")

        except ValueError:
            self.log_message("Invalid coordinates")

    def start_mission(self):
        """Send start mission command to the boat"""
        if not self.connected or not self.gcs_client:
            self.log_message("Not connected")
            return

        if not self.waypoints:
            self.log_message("No waypoints set")
            

        response = self.gcs_client.start_mission()
        if response == "OK":
            self.log_message("Mission started")
        else:
            self.log_message("Failed to start mission")

    def emergency_stop(self):
        """Send emergency stop command to the boat"""
        if not self.connected or not self.gcs_client:
            self.log_message("Not connected")
            return

        self.log_message("EMERGENCY STOP SENT")
        self.gcs_client.emergency_shutdown()

    def update_zoom(self, value):
        """Update the map zoom level"""
        self.map_canvas.set_zoom(int(float(value)))

    def on_close(self):
        """Handle window close event"""
        self.running = False
        if hasattr(self, 'map_canvas'):
            self.map_canvas.cleanup()
        if self.lora:
            if self.gcs_client:
                self.gcs_client.close()
            self.lora.close()
        self.destroy()


if __name__ == "__main__":
    app = GCSApp()
    app.mainloop()
