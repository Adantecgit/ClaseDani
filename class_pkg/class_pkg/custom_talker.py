#################### LIBRERIAS
import rclpy

from math import floor
from threading import Lock, Thread
from time import sleep

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

####################### BATERIA
BATTERY_HIGH = .95
BATTERY_LOW = 0.2  # when the robot will go charge
BATTERY_CRITICAL = 0.1  # when the robot will shutdown

class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')

        self.lock = lock
        self.battery_percent = None
        self.battery_received = False

        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage
            self.battery_received = True
            self.get_logger().info(f'Porcentaje de bateria recivido: {self.battery_percent *100:.2f}%')
            self.destroy_subscription(self.battery_state_subscriber)

    def wait_for_battery(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        while not self.battery_received:
            rclpy.spin_once(self)

        executor.shutdown()

######################## NAVEGACION
def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    navigator = TurtleBot4Navigator()
    battery_percent = None
    position_index = 0

    battery_monitor.wait_for_battery()

    with lock:
        battery_percent = battery_monitor.battery_percent

    if battery_percent is not None:
        navigator.info(f'Porcentaje de batería recibido: {(battery_percent * 100):.2f}% charge')

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Prepare goal pose options
    goal_options = [
        {'name': 'Home', 'pose': navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)},
        {'name': 'Baños', 'pose': navigator.getPoseStamped([-7.55, -6.49], TurtleBot4Directions.SOUTH)},
        {'name': 'Lugar2', 'pose': navigator.getPoseStamped([-0.6, -4.0], TurtleBot4Directions.NORTH)},
        {'name': 'Exit', 'pose': None}
    ]

    navigator.info('Welcome to the mail delivery service.')

    previous_index = 0  # Índice para rastrear la posición anterior

    while True:
        # Crear lista de metas para mostrar
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Pedir al usuario la ubicación deseada
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verificar que el valor ingresado sea un número
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verificar que la selección esté dentro del rango válido
        if selected_index < 0 or selected_index >= len(goal_options):
            navigator.error(f'Goal selection out of bounds: {selected_index}')
        elif goal_options[selected_index]['name'] == 'Exit':
            break
        else:
            # Navegar a la posición solicitada
            navigator.startToPose(goal_options[selected_index]['pose'])

            # Esperar hasta que el robot haya llegado a la posición seleccionada
            navigator.info(f'Robot arrived at {goal_options[selected_index]["name"]}')
            
            # Ahora regresar a la posición anterior
            navigator.info(f'Returning to {goal_options[previous_index]["name"]}')
            navigator.startToPose(goal_options[previous_index]['pose'])

            # Esperar hasta que el robot haya regresado a la posición anterior
            navigator.info(f'Robot returned to {goal_options[previous_index]["name"]}')

            # Actualizar la posición anterior al valor de la selección actual
            previous_index = selected_index

    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
