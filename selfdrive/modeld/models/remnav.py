import pygame, socket, threading, sys, getopt, time
from typing import Dict

options, arguments = getopt.getopt(
    sys.argv[1:],
    'p:',           # short arguments, p with argument
    ["port="])       # long arguments, port with argument

port=6379

for o, a in options:
    if o in ('-p', '--port'):
        try:
            port = int(a)
        except ValueError:
            print("Invalid port: " + a)
            sys.exit()

if len(arguments) < 1:
    arguments = ["192.168.43.1"]
    #arguments=["localhost"]

ADDRESS = (arguments[0], port)

pygame.init()
pygame.display.set_mode((100, 100))

# Indexed by TAG value with the time it was sent.
outstanding_tags : Dict[float,float] = {}       
tag = 0
def new_tag():
    global tag
    tag = tag + 1
    return str(tag)

max_rtt = 0
def process_rtt(rtt):
    global max_rtt
    max_rtt = max(max_rtt, rtt)

def listener(s):
    try:
        while True:
            m = s.recv(1024).decode('utf-8')
            t = time.time()
            for raw_line in m.split('\n'):
                l = raw_line.strip('\r')
                print(f"Received: {l}")
                if l.startswith('<'):
                    # Process the tag
                    tg = l[1:].split('>')[0]
                    if tg in outstanding_tags:
                        # Found it
                        start = outstanding_tags[tg]
                        del outstanding_tags[tg]
                        cur = time.time()
                        t = cur-start
                        process_rtt(t)
                        print(f"({t:.3f}) {l}")
                    else:
                        print(l)
    except socket.error:
        print("Got exception on write")
        quit()            

pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joysticks")
    quit()

j = pygame.joystick.Joystick(0)
j.init()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
print("Waiting to connect to ", ADDRESS)
s.connect(ADDRESS)
print("Connected")

x = threading.Thread(target=listener, args=(s,))
x.start()

linebuffer = ""
v_ratio = 30 # +/-30 degrees steering angle

def do_motion(v):
    v = v * v_ratio
    tg = new_tag()
    cmd = f"<{tg}>s {v:.2f}\r\n"
    print(f"<{tg}> Sending {cmd}")
    s.send(bytes(cmd,"utf-8"))
    outstanding_tags[tg] = time.time()

def handle_key(k):
    global linebuffer, v_ratio
    print(k,end="")
    if k == pygame.K_UP:
        v_ratio = v_ratio + 1
    elif k == pygame.K_DOWN:
        v_ratio = v_ratio - 1
    elif k != "\n":
        linebuffer = linebuffer + k
    elif linebuffer == "?":
        print(f"Max RTT: {int(max_rtt*1000)}mSec")
        print("Steering_Limit: ",v_ratio)
        linebuffer = ""
    else:
        s.send(bytes(linebuffer + "\r\n", "utf-8"))
        linebuffer = ""

try:
    outstanding_tags = {}
    while True:
        event = pygame.event.wait()
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.JOYAXISMOTION:
            do_motion(event.value)
        elif event.type == pygame.KEYDOWN:
            handle_key(event.unicode)
except KeyboardInterrupt:
    pass


