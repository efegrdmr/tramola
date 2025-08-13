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

from tramola.loralib import Lora, LoraGCSClient

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
        
    def clear_waypoints(self):
        """Clear all waypoints from the map"""
        self.waypoints = []
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
        self.geometry("1280x800")  # Increased height to show all controls
        self.minsize(1280, 800)    # Set minimum size to ensure visibility
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Initialize LoRa connection (default values)
        self.lora = None
        self.gcs_client = None
        self.connected = False
        self.update_thread = None
        self.running = False
        self.waypoints = []
        
        # State tracking for mission completion
        self.prev_state = ""
        self.mission_active = False
        
        # Manual control variables
        self.in_manual_mode = False
        self.manual_speed = 0.0
        self.manual_yaw = 0.0
        self.manual_speed_step = 0.1
        self.manual_yaw_step = 0.1
        
        # Color selection
        self.target_color = tk.StringVar(value="RED")

        self.create_widgets()
        self.update_connection_status()
        
        # Bind arrow keys for manual control
        self.bind("<KeyPress>", self.on_key_press)
        self.bind("<KeyRelease>", self.on_key_release)

        # Log that we're starting with Ankara as the map center
        self.log_message("Map initialized to Ankara coordinates")
        
        # Add application timestamp and user info
        self.log_message("Session started by: efegrdmr at 2025-07-31 11:42:06 UTC")

    def create_widgets(self):
        # Create main container with a PanedWindow for resizable sections
        main_paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create left frame for map and console
        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=3)

        # Create right frame with scrollbar support
        right_outer_frame = ttk.Frame(main_paned)
        main_paned.add(right_outer_frame, weight=1)
        
        # Add scrollable frame for the right panel
        right_canvas = tk.Canvas(right_outer_frame, borderwidth=0)
        right_scrollbar = ttk.Scrollbar(right_outer_frame, orient="vertical", command=right_canvas.yview)
        right_scrollable_frame = ttk.Frame(right_canvas)
        
        right_scrollable_frame.bind(
            "<Configure>",
            lambda e: right_canvas.configure(
                scrollregion=right_canvas.bbox("all")
            )
        )
        
        right_canvas.create_window((0, 0), window=right_scrollable_frame, anchor="nw")
        right_canvas.configure(yscrollcommand=right_scrollbar.set)
        
        right_canvas.pack(side="left", fill="both", expand=True)
        right_scrollbar.pack(side="right", fill="y")

        # Map frame
        map_frame = ttk.LabelFrame(left_frame, text="Navigation Map")
        map_frame.pack(fill=tk.BOTH, expand=True, pady=2)

        self.map_canvas = SimpleMapCanvas(map_frame, width=600, height=500)
        self.map_canvas.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        self.lat_entry_var = tk.StringVar()
        self.lon_entry_var = tk.StringVar()

        # Map status bar
        map_status_frame = ttk.Frame(left_frame)
        map_status_frame.pack(fill=tk.X, pady=1)

        ttk.Label(map_status_frame, textvariable=self.map_canvas.status_text).pack(side=tk.LEFT)

        # Status console
        console_frame = ttk.LabelFrame(left_frame, text="Status Console")
        console_frame.pack(fill=tk.X, pady=2)

        self.console = tk.Text(console_frame, height=6, width=40, font=("Courier", 9))
        self.console.pack(fill=tk.BOTH, padx=2, pady=2)
        self.console.config(state=tk.DISABLED)

        # ===== RIGHT PANEL - Two-column layout =====
        # Create a frame for the two columns
        columns_frame = ttk.Frame(right_scrollable_frame)
        columns_frame.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        
        # Create left column for controls
        left_col = ttk.Frame(columns_frame)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=1, pady=1)
        
        # Create right column for controls
        right_col = ttk.Frame(columns_frame)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=1, pady=1)

        # ===== LEFT COLUMN =====
        # Connection controls
        connection_frame = ttk.LabelFrame(left_col, text="Connection")
        connection_frame.pack(fill=tk.X, pady=2)

        conn_settings = ttk.Frame(connection_frame)
        conn_settings.pack(fill=tk.X, padx=2, pady=2)
        
        ttk.Label(conn_settings, text="Port:").grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.port_entry = ttk.Entry(conn_settings, width=8)
        self.port_entry.grid(row=0, column=1, padx=2, pady=2)
        port = "/dev/ttyACM0"
        self.port_entry.insert(0, port)

        ttk.Label(conn_settings, text="Baud:").grid(row=0, column=2, padx=2, pady=2, sticky=tk.W)
        self.baud_entry = ttk.Entry(conn_settings, width=6)
        self.baud_entry.grid(row=0, column=3, padx=2, pady=2)
        self.baud_entry.insert(0, "9600")

        self.connect_btn = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(fill=tk.X, padx=2, pady=2)

        self.status_label = ttk.Label(connection_frame, text="Disconnected", foreground="red")
        self.status_label.pack(fill=tk.X, padx=2, pady=2)
        
        # Boat Status section
        status_frame = ttk.LabelFrame(left_col, text="Boat Status")
        status_frame.pack(fill=tk.X, pady=2)

        # Add position to status labels (compact format)
        status_labels = ["Latitude", "Longitude", "Speed (Real)", "Speed (Req)",
                        "Heading", "Yaw Requested", "Thruster L", "Thruster R", "State"]
        self.status_values = {}

        for i, label in enumerate(status_labels):
            status_row = ttk.Frame(status_frame)
            status_row.pack(fill=tk.X, padx=2, pady=0)
            
            ttk.Label(status_row, text=label + ":", font=("Helvetica", 8)).pack(side=tk.LEFT)
            value_var = tk.StringVar(value="N/A")
            self.status_values[label] = value_var
            ttk.Label(status_row, textvariable=value_var, font=("Helvetica", 8)).pack(side=tk.RIGHT)

        # Manual Control section
        manual_frame = ttk.LabelFrame(left_col, text="Manual Control")
        manual_frame.pack(fill=tk.X, pady=2)
        
        # Manual mode buttons in one row
        manual_btn_frame = ttk.Frame(manual_frame)
        manual_btn_frame.pack(fill=tk.X, pady=2)
        
        self.start_manual_btn = ttk.Button(manual_btn_frame, text="Start Manual", 
                                          command=self.start_manual_mode, width=10)
        self.start_manual_btn.pack(side=tk.LEFT, padx=2)
        
        self.stop_manual_btn = ttk.Button(manual_btn_frame, text="Stop Manual", 
                                         command=self.stop_manual_mode, width=10)
        self.stop_manual_btn.pack(side=tk.RIGHT, padx=2)
        
        # Manual control status
        self.manual_status_var = tk.StringVar(value="Manual Mode: Inactive")
        ttk.Label(manual_frame, textvariable=self.manual_status_var, font=("Helvetica", 8, "bold")).pack(pady=1)
        
        # Manual control values
        manual_values = ttk.Frame(manual_frame)
        manual_values.pack(fill=tk.X, pady=1)
        
        self.manual_speed_var = tk.StringVar(value="Speed: 0.0")
        ttk.Label(manual_values, textvariable=self.manual_speed_var, font=("Helvetica", 8)).pack(side=tk.LEFT, padx=5)
        
        self.manual_yaw_var = tk.StringVar(value="Yaw: 0.0")
        ttk.Label(manual_values, textvariable=self.manual_yaw_var, font=("Helvetica", 8)).pack(side=tk.RIGHT, padx=5)
        
        # Instructions for manual mode - compact
        manual_help = ttk.Frame(manual_frame)
        manual_help.pack(fill=tk.X, pady=1)
        ttk.Label(manual_help, text="↑/↓: Speed | ←/→: Yaw | Space: Stop", 
                font=("Helvetica", 7)).pack()

        # ===== RIGHT COLUMN =====
        # Emergency stop - always visible at top of right column
        emergency_frame = ttk.LabelFrame(right_col, text="Emergency")
        emergency_frame.pack(fill=tk.X, pady=2)
        
        self.emergency_btn = ttk.Button(emergency_frame, text="EMERGENCY STOP",
                                      command=self.emergency_stop, style="Emergency.TButton")
        self.emergency_btn.pack(fill=tk.X, padx=2, pady=2)

        # Configure emergency button style
        self.style = ttk.Style()
        self.style.configure("Emergency.TButton", foreground="white", background="red", font=("Helvetica", 9, "bold"))
        
        # Waypoint Control section
        waypoint_frame = ttk.LabelFrame(right_col, text="Waypoint Control")
        waypoint_frame.pack(fill=tk.X, pady=2)

        # Compact waypoint entry
        wp_entry = ttk.Frame(waypoint_frame)
        wp_entry.pack(fill=tk.X, pady=1)
        
        ttk.Label(wp_entry, text="Lat:").grid(row=0, column=0, padx=2, pady=1, sticky=tk.W)
        self.lat_entry = ttk.Entry(wp_entry, width=9, textvariable=self.lat_entry_var)
        self.lat_entry.grid(row=0, column=1, padx=2, pady=1)

        ttk.Label(wp_entry, text="Lon:").grid(row=0, column=2, padx=2, pady=1, sticky=tk.W)
        self.lon_entry = ttk.Entry(wp_entry, width=9, textvariable=self.lon_entry_var)
        self.lon_entry.grid(row=0, column=3, padx=2, pady=1)

        # Waypoint buttons
        wp_buttons = ttk.Frame(waypoint_frame)
        wp_buttons.pack(fill=tk.X, pady=1)

        self.add_wp_btn = ttk.Button(wp_buttons, text="Add WP", command=self.add_waypoint, width=8)
        self.add_wp_btn.pack(side=tk.LEFT, padx=2)
        
        self.clear_wp_btn = ttk.Button(wp_buttons, text="Clear WPs", command=self.clear_waypoints_vehicle, width=8)
        self.clear_wp_btn.pack(side=tk.RIGHT, padx=2)
        
        # Target Color Selection 
        color_frame = ttk.LabelFrame(right_col, text="Target Color")
        color_frame.pack(fill=tk.X, pady=2)
        
        # Radio buttons in a compact row
        color_row = ttk.Frame(color_frame)
        color_row.pack(fill=tk.X, padx=2, pady=1)
        
        ttk.Radiobutton(color_row, text="RED", variable=self.target_color, 
                      value="RED").pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(color_row, text="GREEN", variable=self.target_color, 
                      value="GREEN").pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(color_row, text="BLACK", variable=self.target_color, 
                      value="BLACK").pack(side=tk.LEFT, padx=2)
        
        # Set color button
        self.set_color_btn = ttk.Button(color_frame, text="Set Target Color", 
                                      command=self.set_target_color)
        self.set_color_btn.pack(fill=tk.X, padx=2, pady=1)
        
        # Mission Control section
        mission_frame = ttk.LabelFrame(right_col, text="Mission Control")
        mission_frame.pack(fill=tk.X, pady=2)
        
        # Start mission button
        self.start_mission_btn = ttk.Button(mission_frame, text="Start Mission", command=self.start_mission)
        self.start_mission_btn.pack(fill=tk.X, padx=2, pady=2)
        
        # Map Controls section
        map_control_frame = ttk.LabelFrame(right_col, text="Map Controls")
        map_control_frame.pack(fill=tk.X, pady=2)

        # Zoom control
        zoom_frame = ttk.Frame(map_control_frame)
        zoom_frame.pack(fill=tk.X, pady=1)
        
        ttk.Label(zoom_frame, text="Zoom:").pack(side=tk.LEFT, padx=2)
        self.zoom_scale = ttk.Scale(zoom_frame, from_=1, to=19, orient=tk.HORIZONTAL,
                                  command=self.update_zoom)
        self.zoom_scale.set(13)  # Default zoom level for OSM
        self.zoom_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Map control buttons - in a row to save space
        btn_frame = ttk.Frame(map_control_frame)
        btn_frame.pack(fill=tk.X, pady=1)
        
        self.center_boat_btn = ttk.Button(btn_frame, text="Center Boat",
                                        command=self.center_on_boat, width=10)
        self.center_boat_btn.pack(side=tk.LEFT, padx=2)

        self.refresh_map_btn = ttk.Button(btn_frame, text="Refresh Map",
                                        command=self.refresh_map_tiles, width=10)
        self.refresh_map_btn.pack(side=tk.RIGHT, padx=2)

        # Toggle map display
        self.show_map_var = tk.BooleanVar(value=PIL_AVAILABLE)
        self.show_map_cb = ttk.Checkbutton(map_control_frame, text="Show Map Tiles",
                                         variable=self.show_map_var,
                                         command=self.toggle_map_display)
        self.show_map_cb.pack(fill=tk.X, padx=2, pady=1)

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

        # Exit manual mode if active
        if self.in_manual_mode:
            self.stop_manual_mode()

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
            self.clear_wp_btn.state(["!disabled"])
            self.start_manual_btn.state(["!disabled"])
            self.stop_manual_btn.state(["!disabled"])
            self.set_color_btn.state(["!disabled"])
        else:
            self.status_label.config(text="Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
            self.start_mission_btn.state(["disabled"])
            self.emergency_btn.state(["disabled"])
            self.add_wp_btn.state(["disabled"])
            self.clear_wp_btn.state(["disabled"])
            self.start_manual_btn.state(["disabled"])
            self.stop_manual_btn.state(["disabled"])
            self.set_color_btn.state(["disabled"])

            # Reset status values
            for var in self.status_values.values():
                var.set("N/A")
                
            # Reset manual mode status
            self.in_manual_mode = False
            self.manual_status_var.set("Manual Mode: Inactive")

    def start_manual_mode(self):
        """Start manual control mode"""
        if not self.connected or not self.gcs_client:
            self.log_message("Not connected")
            return
            
        if self.in_manual_mode:
            self.log_message("Already in manual mode")
            return
            
        response = self.gcs_client.start_manual_mode()
        if response == "OK":
            # Clear waypoints when entering manual mode
     
            self.in_manual_mode = True
            self.manual_status_var.set("Manual Mode: ACTIVE")
            self.log_message("Manual control mode activated")
            self.manual_speed = 0.0
            self.manual_yaw = 0.0
            self.update_manual_display()
            
            # Disable auto mission controls
            self.start_mission_btn.state(["disabled"])
            self.add_wp_btn.state(["disabled"])
        else:
            self.log_message("Failed to start manual mode")
            
    def stop_manual_mode(self):
        """Stop manual control mode"""
        if not self.connected or not self.gcs_client:
            self.log_message("Not connected")
            return
            
        if not self.in_manual_mode:
            self.log_message("Not in manual mode")
            return
            
        # First zero out controls
        self.manual_speed = 0.0
        self.manual_yaw = 0.0
        self.send_manual_control()
        
        response = self.gcs_client.stop_manual_mode()
        if response == "OK":
            self.in_manual_mode = False
            self.manual_status_var.set("Manual Mode: Inactive")
            self.log_message("Manual control mode deactivated")
            
            # Re-enable auto mission controls
            self.start_mission_btn.state(["!disabled"])
            self.add_wp_btn.state(["!disabled"])
        else:
            self.log_message("Failed to stop manual mode")
            
    def send_manual_control(self):
        """Send manual control values to the boat"""
        if self.connected and self.gcs_client and self.in_manual_mode:
            self.gcs_client.send_manual_control_request(self.manual_speed, self.manual_yaw)
            self.update_manual_display()
            
    def update_manual_display(self):
        """Update manual control display values"""
        self.manual_speed_var.set("Speed: {:.2f}".format(self.manual_speed))
        self.manual_yaw_var.set("Yaw: {:.2f}".format(self.manual_yaw))
            
    def on_key_press(self, event):
        """Handle key press events for manual control"""
        if not self.in_manual_mode:
            return
            
        # Process arrow keys
        if event.keysym == 'Up':
            self.manual_speed = min(1.0, self.manual_speed + self.manual_speed_step)
            self.send_manual_control()
        elif event.keysym == 'Down':
            self.manual_speed = max(-1.0, self.manual_speed - self.manual_speed_step)
            self.send_manual_control()
        elif event.keysym == 'Left':
            self.manual_yaw = max(-1.0, self.manual_yaw - self.manual_yaw_step)
            self.send_manual_control()
        elif event.keysym == 'Right':
            self.manual_yaw = min(1.0, self.manual_yaw + self.manual_yaw_step)
            self.send_manual_control()
        elif event.keysym == 'space':
            # Space bar for emergency stop in manual mode
            self.manual_speed = 0.0
            self.manual_yaw = 0.0
            self.send_manual_control()
            self.log_message("Manual controls zeroed")
            
    def on_key_release(self, event):
        """Handle key release events"""
        # Currently not doing anything on key release
        pass
            
    def set_target_color(self):
        """Set the target color for the mission"""
        if not self.connected or not self.gcs_client:
            self.log_message("Not connected")
            return
            
        color = self.target_color.get()
        response = self.gcs_client.set_objective_color(color)
        
        if response == "OK":
            self.log_message("Target color set to: " + color)
        else:
            self.log_message("Failed to set target color")
            
    def clear_waypoints(self):
        """Clear all waypoints from the map and internal list"""
        self.waypoints = []
        self.map_canvas.clear_waypoints()
        self.log_message("All waypoints cleared")

    def clear_waypoints_vehicle(self):
        self.gcs_client.clear_waypoints()
        self.clear_waypoints()


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
                
                # Update state information and detect mission completion
                current_state = ""
                if hasattr(self.gcs_client, 'state') and self.gcs_client.state:
                    current_state = self.gcs_client.state
                    self.status_values["State"].set(current_state)
                
                    # If we started a mission and now the state is idle, clear waypoints
                    if "GOTO" in self.prev_state and current_state.lower().startswith("idle"):
                        if self.mission_active:
                            self.mission_active = False
                            self.after(1000, self.auto_clear_waypoints)  # Schedule waypoint clearing with slight delay
                    
                    # Track if a mission is active
                    if "GOTO" in current_state:
                        self.mission_active = True
                        
                    # Update previous state
                    self.prev_state = current_state

                # Handle thruster values as tuple (left, right)
                if isinstance(self.gcs_client.thruster_requested, tuple) and len(
                        self.gcs_client.thruster_requested) >= 2:
                    self.status_values["Thruster L"].set("{:.1f}%".format(self.gcs_client.thruster_requested[0]))
                    self.status_values["Thruster R"].set("{:.1f}%".format(self.gcs_client.thruster_requested[1]))
                else:
                    self.status_values["Thruster L"].set("N/A")
                    self.status_values["Thruster R"].set("N/A")

            except Exception as e:
                self.log_message("Update error: {}".format(e))

            time.sleep(0.1)
            
    def auto_clear_waypoints(self):
        """Clear waypoints automatically after mission completes"""
        if self.waypoints:
            self.clear_waypoints()
            self.log_message("Mission completed - waypoints cleared automatically")

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
            return

        response = self.gcs_client.start_mission()
        if response == "OK":
            self.log_message("Mission started")
            self.mission_active = True
        else:
            self.log_message("Failed to start mission")

    def emergency_stop(self):
        """Send emergency stop command to the boat"""
        if not self.connected or not self.gcs_client:
            self.log_message("Not connected")
            return

        self.log_message("EMERGENCY STOP SENT")
        self.gcs_client.emergency_shutdown()
        
        # Clear all waypoints on emergency stop
        self.clear_waypoints()
        
        # If in manual mode, also stop manual mode
        if self.in_manual_mode:
            self.stop_manual_mode()

    def update_zoom(self, value):
        """Update the map zoom level"""
        self.map_canvas.set_zoom(int(float(value)))

    def on_close(self):
        """Handle window close event"""
        self.running = False
        
        # Exit manual mode if active
        if self.in_manual_mode and self.connected and self.gcs_client:
            self.stop_manual_mode()
            
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