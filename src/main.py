import mujoco
import glfw
import time
import numpy as np

# Загрузка модели MuJoCo
model = mujoco.MjModel.from_xml_path("/home/anastaisha/PycharmProjects/robo_basketball_project/models/roki.xml")  # Укажите путь к файлу вашей модели
data = mujoco.MjData(model)

# Инициализация окна для визуализации
glfw.init()
window = glfw.create_window(900, 700, "MuJoCo Simulation", None, None)
glfw.make_context_current(window)

# Инициализация камеры, сцены и рендер-контекста
context = mujoco.MjvCamera()
context.azimuth = 90
context.elevation = 0
context.distance = 0.7
context.lookat = (0.1, -0.036, 0.13)
scene = mujoco.MjvScene(model, maxgeom=1000)
option = mujoco.MjvOption()
renderer = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)  # Добавление правильного аргумента

def render_scene():
    mujoco.mjv_updateScene(model, data, option, None, context, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(mujoco.MjrRect(0, 0, 900, 700), scene, renderer)

def move_right_shoulder(position, seconds=1):
    start_time = time.time()  # Запоминаем текущее время
    data.ctrl[model.actuator('right_shoulder_pitch').id] += position
    while time.time() - start_time < seconds:
        mujoco.mj_step(model, data)  # Выполнение шага симуляции
        render_scene()  # Отображение симуляции
        glfw.swap_buffers(window)  # Обновление окна
        glfw.poll_events()
        time.sleep(model.opt.timestep)  # Ждем на время одного шага симуляции (например, 0.01 секунд)

    # data.ctrl[0] = 0.0

def move_right_elbow_yaw(position, seconds=1):
    start_time = time.time()  # Запоминаем текущее время
    data.ctrl[model.actuator('right_elbow_yaw').id] += position
    while time.time() - start_time < seconds:
        mujoco.mj_step(model, data)  # Выполнение шага симуляции
        render_scene()  # Отображение симуляции
        glfw.swap_buffers(window)  # Обновление окна
        glfw.poll_events()
        time.sleep(model.opt.timestep)  # Ждем на время одного шага симуляции (например, 0.01 секунд)

    # data.ctrl[0] = 0.0

def sleep(seconds):
  start_time = time.time()  # Запоминаем текущее время
  while time.time() - start_time < seconds:
    mujoco.mj_step(model, data)  # Выполнение шага симуляции
    render_scene()  # Отображение симуляции
    glfw.swap_buffers(window)  # Обновление окна
    glfw.poll_events()
    time.sleep(model.opt.timestep)

def move_ball(x, y, z):
  ball_id = None
  data.qpos[model.jnt_qposadr[ball_id]:model.jnt_qposadr[ball_id] + 3] = np.array([x, y, z])


model.opt.gravity = (0, 0, 0)
move_right_shoulder(1.57)
sleep(1)
move_right_elbow_yaw(-1.57)
right_elbow_yaw_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'right_elbow_yaw')
print(data.geom_xpos[right_elbow_yaw_id])
sleep(2)
model.opt.gravity = (0, 0, -9.8)
sleep(2)
move_right_shoulder(1.57)
sleep(5)

# Закрытие окна и завершение симуляции
while not glfw.window_should_close(window):
  render_scene()
  glfw.swap_buffers(window)
  glfw.poll_events()

glfw.terminate()