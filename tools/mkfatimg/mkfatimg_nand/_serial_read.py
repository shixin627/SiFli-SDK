import serial, sys, time
port = sys.argv[1] if len(sys.argv) > 1 else 'COM14'
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
try:
    s = serial.Serial(port, 1000000, timeout=0.2)
except Exception as e:
    print('SERIAL_OPEN_FAIL:', e)
    sys.exit(1)
cmd = sys.argv[3] if len(sys.argv) > 3 else None
if cmd:
    s.write(b'\r\n')
    time.sleep(0.2)
    s.write((cmd + '\r\n').encode())
end = time.time() + dur
while time.time() < end:
    data = s.read(8192)
    if data:
        sys.stdout.write(data.decode('utf-8', 'replace'))
        sys.stdout.flush()
s.close()
print('\n[serial-read done]')
