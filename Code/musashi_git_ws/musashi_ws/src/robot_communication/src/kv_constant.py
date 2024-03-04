ERROR = {
    'E0\r\n': 'Device number error.',
    'E1\r\n': 'Command error.',
    'E2\r\n': 'Program unregistered.',
    'E4\r\n': 'Writing prohibition.',
    'E5\r\n': 'Device error.',
    'E6\r\n': 'No comment.'
}

NOT_ERROR = 'OK\r\n'

ENCODER_RIGHT = {
    'dm': 4166,
    'format': 'L'
}
ENCODER_LEFT = {
    'dm': 4168,
    'format': 'L'
}
VELOCITY_RIGHT = {
    'dm': 3080,
    'format': 'L'
}
VELOCITY_LEFT = {
    'dm': 3082,
    'format': 'L'
}
DM_RUNNING = {
    'dm': 3084,
    'format': 'U'
}
DM_HEART_BEAT = {
    'dm': 3085,
    'format': 'U'
}
DM_SERVO_ON = {
    'dm': 4000,
    'format': 'U'
}
DM_RELASE_BRAKE = {
    'dm': 4001,
    'format': 'U'
}
DM_AUTOMATE_RUN = {
    'dm': 4002,
    'format': 'U'
}
DM_SCANNER = {
    'dm': 4003,
    'format': 'U'
}
DM_ULTRA_SONIC = {
    'dm': 4004,
    'format': 'U'
}
DM_DEST = {
    'dm': 4005,
    'format': 'U'
}
DM_BATTERY = {
    'dm': 4006,
    'format': 'U'
}
DM_ROBOT_STATUS = {
    'dm': 4007,
    'format': 'U'
}
