import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path('path/to/so101.xml')
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)