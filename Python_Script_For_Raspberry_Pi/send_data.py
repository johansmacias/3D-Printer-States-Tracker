import json
import notecard
from periphery import I2C
import time
from bluepy.btle import *
import binascii
from datetime import datetime

class MyDelegate(DefaultDelegate):
        def __init__(self):
                DefaultDelegate.__init__(self)

        def handleNotification(self, cHandle, data):
                val = binascii.b2a_hex(tx.read())
                val = binascii.unhexlify(val)
                print(val)
                val = str(val)
                val = val.split(',')
                global device 
                device = val[1]
                global state 
                state = val[2]
                global states
                states = states.fromkeys(states, 0)
                states[val[2]] = 1
                print(states)

productUID = "putYourProductUIDHere"
ble_address = "putTheWisBlockBLEAddressHere"

port = I2C("/dev/i2c-1")
card = notecard.OpenI2C(port, 0, 0)

req = {"req": "hub.set"}
req["product"] = productUID
req["mode"] = "continuous"

print(json.dumps(req))

rsp = card.Transaction(req)
print(rsp)

device = 'generic'
state = 'default'

states =	{
  "Off":        0,
  "Standby":    0,
  "Printing":   0,
}

while True:

        try: 
                print("Trying to connect with the reporter...")
                per = Peripheral(ble_address, "random")
                print("Succesful connection!")
                rx = per.getCharacteristics(uuid='6e400002-b5a3-f393-e0a9-e50e24dcca9e')[0]
                rx.write(b"\xAB")
                time.sleep(5)
                tx = per.getCharacteristics(uuid='6e400003-b5a3-f393-e0a9-e50e24dcca9e')[0]
                val = binascii.b2a_hex(tx.read())
                val = binascii.unhexlify(val)
                print(val)
                val = str(val)
                val = val.split(',')
                device = val[1]
                state = val[2]
                states = states.fromkeys(states, 0)
                states[val[2]] = 1
                break
        except BTLEDisconnectError:
                print("Failed to connect with the reporter. Try again in 15 seconds.")
                time.sleep(15)
        

req = {"req": "note.add"}
req["file"] = "data.qo"
req["start"] = True
req["body"] = states
print(req)
rsp = card.Transaction(req)
print(rsp)

# set callback for notifications
per.setDelegate(MyDelegate())

# enable notification
setup_data = b"\x01\x00"
notify = per.getCharacteristics(uuid='6e400003-b5a3-f393-e0a9-e50e24dcca9e')[0]
notify_handle = notify.getHandle() + 1
per.writeCharacteristic(notify_handle, setup_data, withResponse=True)

last_time = time.time()
send_period = 0

if (states["Printing"] == 1):
        send_period = 5
else:
        send_period = 30

while True:

        if per.waitForNotifications(1.0) or ((time.time() - last_time) > send_period * 60):
                last_time = time.time()
                if (states["Printing"] == 1):
                        send_period = 5
                else:
                        send_period = 30
                req = {"req": "note.add"}
                req["file"] = "data.qo"
                req["start"] = True
                req["body"] = states
                rsp = card.Transaction(req)
                print(rsp)
                continue
