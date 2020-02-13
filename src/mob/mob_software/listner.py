import serial
sp = serial.Serial('/dev/ttyUSB0', 9600)
i = 0

print("listening")                
sp.flushInput()
cm = 'Q\r'.encode()
sp.write(cm)
t = sp.read(1)
if t == '.':
    print('done')
else:
    print('Fuck this shit')
    print(str(t))

sp.flushInput()
cmd = 'QP\r'.encode()
sp.write(cmd)
if sp.read() == '\r':
    print(str(r))
else:
    print("Fuuuuuuuuk")
