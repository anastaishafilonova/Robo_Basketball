import mujoco
import glfw
import time
import numpy as np

# Загрузка модели MuJoCo
model = mujoco.MjModel.from_xml_path("/home/crazer/Robo_Basketball/models/roki.xml")  # Укажите путь к файлу вашей модели
data = mujoco.MjData(model)

# Инициализация окна для визуализации
glfw.init()
window = glfw.create_window(900, 700, "MuJoCo Simulation", None, None)
glfw.make_context_current(window)

# Инициализация камеры, сцены и рендер-контекста
context = mujoco.MjvCamera()
context.azimuth = 90
context.elevation = 0
context.distance = 1.2
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

    #data.ctrl[0] = 0.0

def throw(position, angular_velocity, seconds=1):
    start_time = time.time()  # Запоминаем текущее время
    start = data.ctrl[model.actuator('right_shoulder_pitch').id]
    with open("../data/velocity_distance.txt", "a") as output_file:
        print(angular_velocity, end=" ", file=output_file)
    while data.ctrl[model.actuator('right_shoulder_pitch').id] < start + position:
        data.ctrl[model.actuator('right_shoulder_pitch').id] += angular_velocity * model.opt.timestep
        mujoco.mj_step(model, data)  # Выполнение шага симуляции
        render_scene()  # Отображение симуляции
        glfw.swap_buffers(window)  # Обновление окна
        glfw.poll_events()
        time.sleep(model.opt.timestep)  # Ждем на время одного шага симуляции (например, 0.01 секунд)

    #data.ctrl[0] = 0.0


def move_right_elbow_yaw(position, seconds=1):
    start_time = time.time()  # Запоминаем текущее время
    data.ctrl[model.actuator('right_elbow_yaw').id] += position
    while time.time() - start_time < seconds:
        mujoco.mj_step(model, data)  # Выполнение шага симуляции
        render_scene()  # Отображение симуляции
        glfw.swap_buffers(window)  # Обновление окна
        glfw.poll_events()
        time.sleep(model.opt.timestep)  # Ждем на время одного шага симуляции (например, 0.01 секунд)

    #data.ctrl[0] = 0.0

def sleep(seconds):
  start_time = time.time()  # Запоминаем текущее время
  while time.time() - start_time < seconds:
    mujoco.mj_step(model, data)  # Выполнение шага симуляции
    render_scene()  # Отображение симуляции
    glfw.swap_buffers(window)  # Обновление окна
    glfw.poll_events()
    time.sleep(model.opt.timestep)
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ball')
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    z_ball = data.geom_xpos[ball_id][2]
    z_floor = data.geom_xpos[floor_id][2]
    ball_size = model.geom_size[ball_id][0]
    if z_ball - ball_size <= z_floor:
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'camera')
        x_camera = model.cam_pos[camera_id][0]
        x_ball = data.geom_xpos[ball_id][0]
        depth = round(z_floor - (z_ball - ball_size), 4)
        distance = round(x_ball - x_camera, 3)
        with open("../data/depth_distance.txt", "a") as output_file:
            print(depth, distance, file=output_file)
        with open("../data/velocity_distance.txt", "a") as output_file:
            print(distance, file=output_file)
        break

model.opt.gravity = (0, 0, 0)
move_right_shoulder(1.57)
sleep(1)
move_right_elbow_yaw(-1.57)
sleep(0.7)
model.opt.gravity = (0, 0, -9.8)
sleep(1)
throw(1.57, 3.9)
sleep(1)
model.opt.timestep = 0.001
sleep(30)

# Закрытие окна и завершение симуляции
while not glfw.window_should_close(window):
  render_scene()
  glfw.swap_buffers(window)
  glfw.poll_events()

glfw.terminate()
