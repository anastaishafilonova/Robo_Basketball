import mujoco
import glfw
import time

# Загрузка модели MuJoCo
model = mujoco.MjModel.from_xml_path("/home/crazer/mujoco/model/roki.xml")  # Укажите путь к файлу вашей модели
data = mujoco.MjData(model)

# Инициализация окна для визуализации
glfw.init()
window = glfw.create_window(800, 600, "MuJoCo Simulation", None, None)
glfw.make_context_current(window)

# Инициализация камеры, сцены и рендер-контекста
context = mujoco.MjvCamera()
context.azimuth = 180
context.elevation = -10
context.distance = 0.7
context.lookat = (0, 0.05, -0.05)
scene = mujoco.MjvScene(model, maxgeom=1000)
option = mujoco.MjvOption()
renderer = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)  # Добавление правильного аргумента

def render_scene():
    mujoco.mjv_updateScene(model, data, option, None, context, mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(mujoco.MjrRect(0, 0, 800, 600), scene, renderer)

def move_right_elbow(position, seconds=1):
    start_time = time.time()  # Запоминаем текущее время
    data.ctrl[model.actuator('right_elbow_pitch').id] = position
    while time.time() - start_time < seconds:
        mujoco.mj_step(model, data)  # Выполнение шага симуляции
        render_scene()  # Отображение симуляции
        glfw.swap_buffers(window)  # Обновление окна
        glfw.poll_events()
        time.sleep(model.opt.timestep)  # Ждем на время одного шага симуляции (например, 0.01 секунд)

    data.ctrl[0] = 0.0

def move_right_shoulder(position, seconds=1):
    start_time = time.time()  # Запоминаем текущее время
    data.ctrl[model.actuator('right_shoulder_pitch').id] = position
    while time.time() - start_time < seconds:
        mujoco.mj_step(model, data)  # Выполнение шага симуляции
        render_scene()  # Отображение симуляции
        glfw.swap_buffers(window)  # Обновление окна
        glfw.poll_events()
        time.sleep(model.opt.timestep)  # Ждем на время одного шага симуляции (например, 0.01 секунд)

    data.ctrl[0] = 0.0

move_right_elbow(-1)
move_right_shoulder(-1)

# Закрытие окна и завершение симуляции
while not glfw.window_should_close(window):
    render_scene()
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
