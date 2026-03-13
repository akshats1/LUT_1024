import serial
import time

class Motor:
    """
    Duet motor driver with CLOSED-LOOP software Z tracking.

    Key change: move_xyz_u() now has a `wait` parameter.
    - wait=True  (default): sends G1 + M400, blocks until complete. Use for Z moves.
    - wait=False           : sends G1 only, returns immediately. Use for Y moves
                             when you want to pipeline the move with the next
                             frame's settle window.

    Call flush_move() explicitly when you need to wait for a non-blocking move.
    """

    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port     = port
        self.baudrate = baudrate
        self.ser      = None

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.connect_serial()
        self.send_gcode("G91", wait=True)   # Relative mode

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(0.1)
            print("[Motor] Serial connected")
        except serial.SerialException as e:
            print("[Motor ERROR]", e)
            raise RuntimeError("Failed to open motor serial port")

    def release(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_gcode(self, cmd, wait=False):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()
        if wait:
            self.ser.write(b"M400\n")
            self.ser.flush()

    def flush_move(self):
        """
        Wait for the Duet planner to drain any previously issued non-blocking
        G1 moves. Call this before the next capture when using wait=False moves.
        """
        self.ser.write(b"M400\n")
        self.ser.flush()

    def home_all(self):
        print("[Motor] Homing X Y Z")
        self.send_gcode("G28 Z", wait=True)
        self.send_gcode("G28 X Y", wait=True)
        self.send_gcode("G91", wait=True)
        self.z = 0.0
        self.x = 0.0
        self.y = 0.0

    def move_xyz_u(self, x=0, y=0, z=0, u=0,
                   feedrate_xy=300, feedrate_z=100, wait=True):
        """
        Issue a relative G1 move.

        Args:
            wait: If True (default), appends M400 so the call blocks until
                  the move is physically complete. Set wait=False for Y moves
                  that you want to pipeline; call flush_move() later.
        """
        if x == 0 and y == 0 and z == 0 and u == 0:
            return

        cmd = "G1"
        if x: cmd += f" X{x:.5f}"
        if y: cmd += f" Y{y:.5f}"
        if z:
            cmd += f" Z{z:.5f}"
            self.z += z
        if u: cmd += f" U{u:.5f}"

        cmd += f" F{feedrate_z if z != 0 else feedrate_xy}"
        self.send_gcode(cmd, wait=wait)