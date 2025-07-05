import sys, select, termios, tty

def KeyboardInterface(key):
    try:
        if key == 'w':
            keyboard.control.x += 0.1
        elif key == 's':
            keyboard.control.x -= 0.1
        elif key == 'a':
            keyboard.control.yaw += 0.1
        elif key == 'd':
            keyboard.control.yaw -= 0.1
        elif key == ' ':
            keyboard.control.x = 0
            keyboard.control.yaw = 0
    except AttributeError:
        pass

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin],[],[],0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardControl():
    def __init__(self):
        self.control = self.Control()

    class Control:
        def __init__(self):
            self.x = 0.
            self.yaw = 0.

if __name__ == "__main__":
    keyboard = KeyboardControl()

    settings = termios.tcgetattr(sys.stdin)

    while True:
        print("\r" + f"Controller x: {keyboard.control.x:.1f} yaw: {keyboard.control.yaw:.1f}", end='  ', flush=True)
        key = get_key(settings)
        KeyboardInterface(key)
        if key == '\x03':
            print('')
            break