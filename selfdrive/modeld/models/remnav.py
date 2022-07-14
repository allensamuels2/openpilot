import pygame, socket, threading, sys, getopt, math, time

options, arguments = getopt.getopt(
    sys.argv[1:],
    'p:',           # short arguments, p with argument
    ["port="])       # long arguments, port with argument

port=6379

for o, a in options:
    if o in ('-p', '--port'):
        try:
            port = int(a)
        except:
            print("Invalid port: " + a)
            sys.exit()

if len(arguments) < 1:
    arguments = ["172.20.10.10"]

ADDRESS = (arguments[0], port)

pygame.init()

def listener(s):
    try:
        while True:
            m = s.recv(1024);
            print(m.decode('utf-8'))
    except:
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

def do_motion(v):
    v = v * 30  # +/-30 degrees steering angle
    cmd = f"S {v:.2f}\r\n"
    print("Sending cmd: " + cmd)
    s.send(bytes(cmd,"utf-8"))

run = True
while run:
    time.sleep(1)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.JOYAXISMOTION:
            do_motion(event.value)

pygame.quit()
quit()
