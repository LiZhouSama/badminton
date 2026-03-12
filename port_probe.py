import time
import serial
from serial.tools import list_ports

ports=[p.device for p in list_ports.comports() if p.device.upper().startswith('COM')]
print('ports:',ports)

for port in sorted(ports):
    print('\n===',port,'===')
    try:
        s=serial.Serial(port,115200,timeout=0.2)
        print('open ok')
    except Exception as e:
        print('open fail:',repr(e))
        continue

    data=b''
    t0=time.time()
    while time.time()-t0<1.5:
        try:
            data += s.read(s.in_waiting or 1)
        except Exception as e:
            print('read fail:',repr(e))
            break
    s.close()
    print('bytes:',len(data),'head:',data[:24].hex())
