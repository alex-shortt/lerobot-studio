import mujoco
import math
import time
import glfw

# load model
model = mujoco.MjModel.from_xml_path("./so101/so101_new_calib.xml")  # or .urdf
data = mujoco.MjData(model)

# figure out which actuator/joint corresponds to motor_id 6
# you'll need to inspect the model to get the right index or name
JOINT_NAME = "shoulder_pan"  # check the xml
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
actuator_id = 0  # or look it up similarly

# sine params (match your real script)
frequency = 0.5
amplitude = math.radians(30)  # mujoco uses radians
center = 0.0
duration = 10.0

# Initialize GLFW
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

# Create window
window = glfw.create_window(1200, 900, "MuJoCo Simulation", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create window")

glfw.make_context_current(window)

# Create renderer
scene = mujoco.MjvScene(model, maxgeom=10000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
option = mujoco.MjvOption()
perturb = mujoco.MjvPerturb()

# Camera
cam = mujoco.MjvCamera()
cam.lookat[:] = [0, 0, 0]
cam.distance = 5.0
cam.azimuth = 90
cam.elevation = -20

# Set up window close callback
def window_close_callback(window):
    pass  # Just allow closing

glfw.set_window_close_callback(window, window_close_callback)

# Main loop
start = time.time()
while not glfw.window_should_close(window):
    elapsed = time.time() - start
    
    if duration > 0 and elapsed >= duration:
        break
    
    # Calculate sine wave target
    target = center + amplitude * math.sin(2 * math.pi * frequency * elapsed)
    data.ctrl[actuator_id] = target
    
    # Step simulation
    mujoco.mj_step(model, data)
    
    # Render
    width, height = glfw.get_framebuffer_size(window)
    viewport = mujoco.MjrRect(0, 0, width, height)
    
    mujoco.mjv_updateScene(
        model, data, option, perturb, cam, mujoco.mjtCatBit.mjCAT_ALL, scene
    )
    
    mujoco.mjr_render(viewport, scene, context)
    
    glfw.swap_buffers(window)
    glfw.poll_events()
    
    time.sleep(0.02)  # ~50hz

glfw.terminate()