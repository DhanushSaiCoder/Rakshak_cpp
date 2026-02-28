# Kp scales
KP_SCALE_YAW = 10
KP_SCALE_PITCH = 10

REVOLUTION = 360
YAW_REDUCTION=27.0
PITCH_REDUCTION= 192.0 #64.0
CAM_REDUCTION=30 
YAW_VELOCITY_LIMIT = 7.19
YAW_ACCELERATION_LIMIT = 7.19
PITCH_VELOCITY_LIMIT = 51.2
PITCH_ACCELERATION_LIMIT = 51.2
YAW_POS_CONST = 1
PITCH_POS_CONST = 1
YAW_LIMIT = 34  # degrees of 1x horizontal FOV
PITCH_LIMIT = 23  # degrees of 1x vertical FOV

TOP_SERIAL = "USB3Neo_09936"#"USB3Neo_09248"
STATIC_SERIAL = "USB3Neo_09247"#"USB3Neo_09247"
LEFT_SERIAL = "USB3Neo_09936"  #"USB3Neo_09247" #"USB3Neo_09936"
RIGHT_SERIAL = "USB3Neo_07703"
THERMAL_SERIAL = "TV251964YF003"
TRIGGER_PORT = "/dev/arduino_trigger"
SHORT_HOLD_TIME = 2   
LONG_HOLD_TIME  = 6
SHORT_BURST_TIME = 1  # In s
LONG_BURST_TIME = 3  # In s
KNOWN_HEIGHT = 1.7  # In meters

# Model paths
MODELS_DIR = "models"
DEFAULT_MODEL = "models/yolo11n.pt"
FACE_MODEL = "models/yolov11n-face.pt"
PERSON_MODEL = "models/yolo11n.pt"  # models/yolo11n.pt our model trained -> /home/bhairavrobotics/Downloads/best.pt
FIG_MODEL = "models/fig11_best_11n_12k_without_params.pt"


# Sensor data variables
DATA_FORMAT = '<ii10f'
SYNC_WORD = b'START'
BAUD_RATE = 250000
SENSORS_PORT = '/dev/arduino_sensors'

# Depth data loading
TARGET_TYPE = "person"  #Available  ----> "person", "target"

# MQ Base Variables
BASE = "rakshak"
SYSTEM_ID = "system1"
LOGGER_NAME = "basemq" # name assigned when created a logger instance to view all levels of logs 
RABBIT_EXCHANGE = "tracking.events" # all the messsages are available in this exchange 
RABBIT_EXCHANGE_TYPE = "topic" # type : topic available : fanout, direct, default
QUEUE_PLACEHOLDER = "test.q" # a placeholder which will be changes when created an instance
CONNECTION_NAME = "basemq-client" # Optional for WEB view for user in RabbitMQ’s admin tools and logs
PREFETCH = 64 # No of msgs to be sent without acknowledgemnt 
RABBIT_HOST = "192.168.1.100" # Rabbit mq current system ip 
RABBIT_PORT = 5672 # Rabbit mq port always INT
RABBIT_USER = "jetson" # Rabbit mq user name
RABBIT_PASS = "123" # Rabbit mq password
RABBIT_VHOST = "/" # Virtual Host can create a particular host to route all msgs to that path 
RECONNECTION_TIME_LIMIT = 30 # timer to reconnect if time is 30 sec reconnects on every (1,2,4,8,16)
HEARTBEAT = 30 # a pulse if stayed idle to maintain channel alive
BLOCKED_CONNECTION_TIMEOUT = 60 # Maximum time (seconds) that a connection may be blocked before pika forcibly closes it.
SOCKET_TIMEOUT = 10 # Timeout for TCP socket operations (connect, read, write).
CONNECTION_ATTEMPTS = 10 # How many times pika tries to establish the connection before giving up.
RETRY_DELAY = 2 # Time (seconds) to wait between failed connection attempts.

# Data for Position based Calibration based on height 
POS_DATA = {
    1: {
        "height": [140.090515136719, 75.5969848632813, 47.7028198242188,39.6978149414062],
        "yaw_pos": [0.956710815429688, 0.428009033203125, 0.110870361328125,-0.0030059814453125],
        "pitch_pos": [-5.06361389160156, -2.7230224609375, -1.34123229980469,-0.691757202148438]
    },
    2: {
        "height": [228.324096679688,133.331817626953,93.38818359375,76.8417358398438],  
        "yaw_pos": [-0.388565063476562,-0.710693359375,-0.920806884765625,-0.978866577148438],
        "pitch_pos": [-4.5792236328125,-2.49620056152344,-1.54637145996094,-1.05845642089844]
    },
    3: {
        "height": [],  
        "yaw_pos": [],
        "pitch_pos": []
    },
}

# bounding_box_height	motor_yaw_position	motor_pitch_position
# 39.6978149414062	-0.0030059814453125	-0.691757202148438
# 47.7028198242188	0.110870361328125	-1.34123229980469
# 75.5969848632813	0.428009033203125	-2.7230224609375
# 140.090515136719	0.956710815429688	-5.06361389160156

