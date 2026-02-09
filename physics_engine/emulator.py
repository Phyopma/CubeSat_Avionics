import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import threading
import socket
import struct
import time
import math
import numpy as np

# --- Configuration ---
HOST_IP = "127.0.0.1"
HOST_PORT = 5555
WINDOW_WIDTH = 1024
WINDOW_HEIGHT = 768

class AdcsVisualizer:
    def __init__(self):
        self.running = True
        self.app = gui.Application.instance
        self.app.initialize()

        self.window = self.app.create_window("ADCS Emulator", WINDOW_WIDTH, WINDOW_HEIGHT)
        self.w = self.window
        
        # 3D Scene
        self.widget3d = gui.SceneWidget()
        self.widget3d.scene = rendering.Open3DScene(self.w.renderer)
        self.widget3d.scene.set_background([0.1, 0.1, 0.1, 1.0])  # Dark Gray
        
        # Sidebar
        self.panel = gui.Vert(0, gui.Margins(0.25 * self.w.theme.font_size, 0.25 * self.w.theme.font_size, 0.25 * self.w.theme.font_size, 0.25 * self.w.theme.font_size))
        
        # Controls / Status
        self.lbl_status = gui.Label("Status: Waiting for Data...")
        self.lbl_mode = gui.Label("Mode: -")
        self.lbl_omega = gui.Label("Omega: - rad/s")
        self.lbl_time = gui.Label("Sim Time: 0.00 s")

        self.panel.add_child(gui.Label("ADCS Telemetry"))
        self.panel.add_child(self.lbl_status)
        self.panel.add_child(self.lbl_time)
        self.panel.add_child(self.lbl_mode)
        self.panel.add_child(self.lbl_omega)
        
        # Legend
        self.panel.add_child(gui.Label("")) # Spacer
        self.panel.add_child(gui.Label("Legend:"))
        lbl_trq = gui.Label("Red: Torque")
        lbl_trq.text_color = gui.Color(1.0, 0.0, 0.0)
        self.panel.add_child(lbl_trq)
        
        lbl_mag = gui.Label("Yel: B-Field")
        lbl_mag.text_color = gui.Color(1.0, 1.0, 0.0)
        self.panel.add_child(lbl_mag)
        
        lbl_omg = gui.Label("Cyn: Omega")
        lbl_omg.text_color = gui.Color(0.0, 1.0, 1.0)
        self.panel.add_child(lbl_omg)
        
        # Layout
        self.layout = gui.Horiz(0, gui.Margins(0,0,0,0))
        self.layout.add_child(self.panel)
        self.layout.add_child(self.widget3d)
        self.w.add_child(self.layout)

        # Scene Objects
        self.setup_scene()
        
        # Network State
        self.latest_state = None
        self.lock = threading.Lock()
        
        # Start Network Thread
        self.thread = threading.Thread(target=self.network_loop)
        self.thread.daemon = True
        self.thread.start()

        # Update Loop
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_key(self._on_key)
        self.window.set_on_tick_event(self._on_tick)
        self.app.run()

    def _on_key(self, event):
        if event.type == gui.KeyEvent.UP:
            return False
            
        # Camera resets
        if event.key == gui.KeyName.X:
            self.widget3d.look_at([0,0,0], [2, 0, 0], [0, 0, 1])
            return True
        if event.key == gui.KeyName.Y:
            self.widget3d.look_at([0,0,0], [0, 2, 0], [0, 0, 1])
            return True
        if event.key == gui.KeyName.Z:
            self.widget3d.look_at([0,0,0], [0, 0, 2], [0, 1, 0])
            return True
        return False

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.layout.frame = r
        self.panel.frame = gui.Rect(r.x, r.y, 200, r.height)
        self.widget3d.frame = gui.Rect(r.x + 200, r.y, r.width - 200, r.height)

    def _on_tick(self):
        # This is where we update the 3D scene and labels
        self.update_visualization()
        return True # Continue running

    def setup_scene(self):
        # 1. Satellite (2U CubeSat)
        # Dimensions: 10cm x 10cm x 20cm (0.1 x 0.1 x 0.2 units)
        self.create_cubesat_model()

        # 2. Arrows
        self.arrow_torque_name = "arrow_torque"
        self.arrow_mag_name = "arrow_mag"
        self.arrow_omega_name = "arrow_omega"
        
        self.create_arrow(self.arrow_torque_name, [1, 0, 0, 1])   # Red (Torque)
        self.create_arrow(self.arrow_mag_name, [1, 1, 0, 1])      # Yellow (B-Field)
        self.create_arrow(self.arrow_omega_name, [0, 1, 1, 1])    # Cyan (Angular Velocity)

        # 3. Axis Labels (X, Y, Z attached to body)
        self.lbl_x = self.widget3d.add_3d_label([0.1, 0, 0], "X (Body)")
        self.lbl_y = self.widget3d.add_3d_label([0, 0.1, 0], "Y (Body)")
        self.lbl_z = self.widget3d.add_3d_label([0, 0, 0.2], "Z (Body)")
        
        # 4. Inertial Frame (Fixed at origin)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
        self.widget3d.scene.add_geometry("inertial_axes", axes, rendering.MaterialRecord())

        # 5. Earth Background
        # Place it far in -Z (Nadir) direction. Radius 20 units (20m scale relative to 0.1m sat).
        earth = o3d.geometry.TriangleMesh.create_sphere(radius=20.0)
        earth.compute_vertex_normals()
        earth.translate([0, 0, -25.0]) 
        mat_earth = rendering.MaterialRecord()
        mat_earth.shader = "defaultLit"
        mat_earth.base_color = [0.1, 0.2, 0.8, 1.0] # Blue-ish
        self.widget3d.scene.add_geometry("earth", earth, mat_earth)

        # 6. Camera
        self.widget3d.setup_camera(60, o3d.geometry.AxisAlignedBoundingBox([-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]), [0, 0, 0])
        self.widget3d.look_at([0,0,0], [1.0, 1.0, 1.0], [0, 0, 1])
        
        # Prevent mouse from messing up too much (Set to Rotate Camera)
        self.widget3d.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

    def create_cubesat_model(self):
        # Create a merge of geometries for a 2U CubeSat
        # Body: 10x10x20cm Gold
        body = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.2)
        body.compute_vertex_normals()
        body.translate([-0.05, -0.05, -0.1]) # Center it
        body.paint_uniform_color([0.8, 0.7, 0.2]) # Gold-ish

        # Solar Panels (Black) on +X, -X, +Y, -Y faces
        # We can implement this by adding slightly larger, thin plates or just painting.
        # Let's add thin plates for visual depth.
        
        panels = []
        # Side Panels (10x20cm)
        for x, y, z in [(0.051, 0, 0), (-0.051, 0, 0), (0, 0.051, 0), (0, -0.051, 0)]:
            p = o3d.geometry.TriangleMesh.create_box(width=0.002, height=0.09, depth=0.19)
            p.compute_vertex_normals()
            p.translate([-0.001, -0.045, -0.095]) # Center local
            
            # Rotate and translate to face
            if x != 0:
                p.rotate(o3d.geometry.get_rotation_matrix_from_xyz([0, np.pi/2, 0]), center=[0,0,0])
            p.translate([x, y, 0])
            p.paint_uniform_color([0.1, 0.1, 0.1]) # Dark Grey/Black
            panels.append(p)

        # Merge
        combined = body
        for p in panels:
            combined += p
        
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        
        self.widget3d.scene.add_geometry("sat_model", combined, mat)

    def create_arrow(self, name, color, radius=0.015):
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.base_color = color
        
        # Default arrow pointing up Z. Initial size doesn't matter much as we scale it.
        # But cone/cylinder ratio matters.
        arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=radius, cone_radius=radius*2.0, cylinder_height=1.0, cone_height=0.2)
        arrow.compute_vertex_normals()
        self.widget3d.scene.add_geometry(name, arrow, mat)

    def network_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((HOST_IP, HOST_PORT))
        print(f"Listening on {HOST_IP}:{HOST_PORT}...")
        
        while self.running:
            try:
                data, _ = sock.recvfrom(1024)
                if len(data) == 64:
                    # Unpack
                    # <dfffffffffffffi (doub le, 13 floats, 1 int)
                    unpacked = struct.unpack("<dfffffffffffffi", data)
                    
                    with self.lock:
                        self.latest_state = {
                            't': unpacked[0],
                            'q': unpacked[1:5],      # w, x, y, z
                            'w': unpacked[5:8],      # x, y, z
                            'trq': unpacked[8:11],   # x, y, z
                            'b': unpacked[11:14],    # x, y, z
                            'mode': unpacked[14]
                        }
            except Exception as e:
                print(f"Network Error: {e}")

    def update_visualization(self):
        state = None
        with self.lock:
            if self.latest_state:
                state = self.latest_state
        
        if not state:
            return

        # Update Labels
        self.lbl_status.text = "Status: Connected"
        self.lbl_time.text = f"Sim Time: {state['t']:.2f} s"
        self.lbl_omega.text = f"Omega: {np.linalg.norm(state['w']):.3f} rad/s"
        mode_map = {0: "Idle", 1: "Detumble", 2: "Spin", 3: "Pointing"}
        self.lbl_mode.text = f"Mode: {mode_map.get(state['mode'], 'Unknown')}"
        
        # Update Satellite Rotation
        # Open3D uses rotation matrices. Convert Quaternion [w, x, y, z] to Matrix.
        q = state['q'] # [w, x, y, z]
        # Open3D expects [w, x, y, z] for get_rotation_matrix_from_quaternion? 
        # Actually o3d.geometry.get_rotation_matrix_from_quaternion takes (w, x, y, z)
        R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
        
        # Update Cube Transform
        # Note: Open3D GUI scene graph is a bit complex. 
        # For simple 'add_geometry', we might need to remove and re-add or use 'set_geometry_transform' if available in this API.
        # Check rendering.Open3DScene API: has set_geometry_transform(name, transform)
        
        transform = np.identity(4)
        transform[:3, :3] = R
        self.widget3d.scene.set_geometry_transform("sat_model", transform)
        
        # Vector visualization:
        # 1. Torque (Red) - Very small magnitude ~1e-6. Need HUGE scale.
        # 2. B_Field (Yellow) - ~50e-6. Need Large Scale.
        # 3. Omega (Cyan) - ~0.05. Need Medium Scale.
        
        # Torque (Red) -> Scale 200,000 for 1e-6 to become 0.2 units
        self.update_arrow(self.arrow_torque_name, state['trq'], R, scale=200000.0) 
        
        # Mag Field (Yellow) -> Scale 10,000 for 50e-6 to become 0.5 units
        self.update_arrow(self.arrow_mag_name, state['b'], R, scale=10000.0)
        
        # Angular Velocity (Cyan) -> Scale 10.0 for 0.05 to become 0.5 units
        self.update_arrow(self.arrow_omega_name, state['w'], R, scale=10.0)

        # Update Labels (X, Y, Z axes of the body)
        # We want them at the tips of the body axes.
        # 2U body is approx +/- 0.05 in X,Y and +/- 0.1 in Z.
        # Let's place labels slightly outside.
        x_pos = R @ np.array([0.15, 0, 0])
        y_pos = R @ np.array([0, 0.15, 0])
        z_pos = R @ np.array([0, 0, 0.25])
        
        try:
            self.lbl_x.position = x_pos
            self.lbl_y.position = y_pos
            self.lbl_z.position = z_pos
        except AttributeError:
            # Fallback for older/different API: remove and re-add
            # Note: SceneWidget might not have remove_3d_label exposed in all versions.
            # If this fails, we might skip label updates or finding another way.
            # Assuming remove_3d_label might exist or we just ignore updating if it fails.
            try:
                self.widget3d.remove_3d_label(self.lbl_x)
                self.widget3d.remove_3d_label(self.lbl_y)
                self.widget3d.remove_3d_label(self.lbl_z)
                self.lbl_x = self.widget3d.add_3d_label(x_pos, "X (Body)")
                self.lbl_y = self.widget3d.add_3d_label(y_pos, "Y (Body)")
                self.lbl_z = self.widget3d.add_3d_label(z_pos, "Z (Body)")
            except:
                pass # Give up on labels if both fail to avoid crash

        # Force redraw
        self.window.post_redraw()

    def update_arrow(self, name, vector, sat_rotation, scale=1.0):
        # Vector magnitude
        mag = np.linalg.norm(vector)
        if mag < 1e-9:
            # Hide or shrink
            transform = np.identity(4)
            transform[0,0] = 0.0 # Scale to 0
            self.widget3d.scene.set_geometry_transform(name, transform)
            return

        # Direction
        val = np.array(vector) / mag
        
        # Align default Z-axis arrow to 'val'
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, val)
        axis_len = np.linalg.norm(axis)
        
        if axis_len < 1e-6:
            # Parallel or anti-parallel
            if np.dot(z_axis, val) > 0:
                R_vec = np.identity(3)
            else:
                R_vec = -np.identity(3) # Verify this flip
                R_vec[0,0] = 1 # Keep x? No, this is tricky. 180 deg rotation.
                R_vec = o3d.geometry.get_rotation_matrix_from_axis_angle([np.pi, 0, 0])
        else:
            axis = axis / axis_len
            angle = math.acos(np.dot(z_axis, val))
            R_vec = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            
        # Composite Rotation: R_total = R_sat @ R_vec
        # Because we want the vector to be relative to the satellite body?
        # If 'vector' is in Body Frame, then yes.
        # If 'vector' is in Inertial Frame, then just R_vec.
        # Physics engine 'torque' is 'torque_ext_body' (Body Frame).
        # Physics engine 'b_field' is 'B_body' (Body Frame).
        # So yes, rotate by R_sat.
        
        # R_final = sat_rotation @ R_vec
        R_final = sat_rotation @ R_vec
        
        transform = np.identity(4)
        transform[:3, :3] = R_final
        
        # Scale: We want to scale the LENGTH (Z) by mag * scale
        # But for sphere/arrow mesh, uniform scaling is safer usually for transform matrix
        # Let's simple apply uniform scale for now.
        scale_factor = min(mag * scale, 10.0) # Cap size
        
        # Ensure it is at least visible if > 0
        if scale_factor < 0.01: scale_factor = 0.01

        transform[:3, :3] *= scale_factor
        
        self.widget3d.scene.set_geometry_transform(name, transform)


if __name__ == "__main__":
    AdcsVisualizer()
