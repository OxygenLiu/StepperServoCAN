#!/usr/bin/env python3

"""
For greatest compatibility this script doesn't depend bob-built-in modules - 
except for the USB CAN interface (Panda, ) and it can be run on PC or Comma EON. 
Keyboard input (w,s, q keys) in game mode - supported even over SSH
"""
from panda import Panda  # install https://github.com/commaai/panda

import binascii
import argparse
import time
import queue
import threading
import subprocess
from pynput import keyboard


def heartbeat_thread(p):
  while True:
    try:
      p.send_heartbeat()
      time.sleep(1)
    except:
      break
      raise

# from Msg.h which is from ocelot_controls.dbc
MSG_STEERING_COMMAND_FRAME_ID = 0x22e
MSG_STEERING_STATUS_FRAME_ID = 0x22f
motor_bus_speed = 500  #StepperServoCAN baudrate 500kbps
MOTOR_MSG_TS = 0.008 #10Hz
MAX_TORQUE = 32 #0.5 # 0.07Nm/0.4A *1.1A = 0.19Nm x 80% = 0.15Nm
MAX_ANGLE = 3600 

#game mode options
torque_rise_factor = 1.2
torque_decay_factor = 0.8
angle_rise_factor = 1.4
quick_decay_factor = 0.6 #common

#STEERING_COMMAND:STEER_MODE options
modes = {
    'OFF': 0,
    'TORQUE_CONTROL': 1,
    'ANGLE_CONTROL': 2,
    'SOFT_OFF': 3
}

actions = {
    'UP': 1,
    'DOWN': 2,
    'STOP': 3,
    'HOLD': 4
}

def calc_checksum_8bit(work_data, msg_id): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8); #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

import struct
def steering_msg_cmd_data(counter: int, steer_mode: int, steer_torque: float, steer_angle: float) -> bytes:
  # Define the structure format and pack the data
  canbus_fmt = '<Bhb' # msg '<bbhb'  without checksum byte[0]
  packed_data = struct.pack(canbus_fmt,
                            (steer_mode << 4) | counter,
                            max(min(int(steer_angle  * 8), 32767), -32768),
                            max(min(int(steer_torque * 8), 127), -128)
                            )
  checksum = calc_checksum_8bit(packed_data, MSG_STEERING_COMMAND_FRAME_ID)
  packed_data = struct.pack('<B', checksum) + packed_data # add checksum byte at the end
  return packed_data

def CAN_tx_thread(p:Panda, bus):
  print("Starting CAN TX thread...")
  global _torque
  global _angle
  global _mode
  #p.can_clear(bus)
  cnt_cmd = 0
  dat = steering_msg_cmd_data(cnt_cmd, _mode, _torque, _angle)
  p.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, bus)
  
  t_prev =0
  while True:
    time.sleep(MOTOR_MSG_TS)
    cnt_cmd += 1
    dat = steering_msg_cmd_data(cnt_cmd % 0xF, _mode, _torque, _angle)
    p.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, bus)
    

def CAN_rx_thread(p:Panda, bus):
  t_status_msg_prev =0
  print("Starting CAN RX thread...")
  #p.can_clear(bus)     #flush the buffers
  while True:
    time.sleep(MOTOR_MSG_TS/10) #read fast enough so the CAN interface buffer is cleared each loop
    t = time.time()
    can_recv = p.can_recv()
    for address, _, dat, src in can_recv:
      if src == bus and address == MSG_STEERING_STATUS_FRAME_ID:
        if t - t_status_msg_prev > 0.0001:
          hz = 1/(t - t_status_msg_prev)
        else:
          hz = -1
        #print(f"{hz:3.0f}Hz, addr: {address}, bus: {bus}, dat: {binascii.hexlify(dat)}")
        t_status_msg_prev = t

def rise_and_decay(value:float, delta:float, max_min_limit:float):
  small_value = 0.05*max_min_limit # 5% of max_min_limit
  decay = False
  if delta > 1:
    value += small_value
  elif delta < -1:
    value -= small_value
  else:
    decay = True

  if value*delta > 0 or decay:  #same sign
    value = value * abs(delta)
  else: #if direction change, use quick decay
    value = value * quick_decay_factor 
  
  # bound value within max_min_limit
  value = max(min(value, max_min_limit), -max_min_limit)
  
  if abs(value) < small_value: #if value is small, set to 0
    value = 0.0

  return value

def print_cmd_state():
  global _torque
  global _angle
  global _mode
  if _mode == modes['TORQUE_CONTROL']:
    print(f"Torque: {_torque:3.2f}", end='\r') # end='\r' to avoid flooding on console
  elif _mode == modes['ANGLE_CONTROL']:
    print(f"Angle:{_angle:4.2f}, FeedForward torque: {_torque:3.2f}", end='\r')

def on_press(key):
  global _torque
  global _angle
  global _mode

  try: 
    match key.char:
      case 'q': 
        return False
      case 'm': #mode input mode
        _mode = (_mode + 1)%len(modes) # cycle thru modes
        print(f"Mode: {[name for name, val in modes.items() if val == _mode][0]}")
        #_torque = rise_and_decay(_torque, torque_decay_factor, MAX_TORQUE)
        _torque = 0.0
        time.sleep(0.1)
      case 'w' if _mode == modes['TORQUE_CONTROL']: # in wsad game mode, torque is controlled by keyboard and ramp generator
        _torque = rise_and_decay(_torque, torque_rise_factor, MAX_TORQUE)
      case 's' if _mode == modes['TORQUE_CONTROL']: 
        _torque = rise_and_decay(_torque, -torque_rise_factor, MAX_TORQUE)
      case 'd' if _mode == modes['ANGLE_CONTROL']:  
        _angle = rise_and_decay(_angle, angle_rise_factor, MAX_ANGLE)
        _torque = max(abs(_torque), 0.1*MAX_TORQUE) #match torque signal to angle
      case 'a' if _mode == modes['ANGLE_CONTROL']: 
        _angle = rise_and_decay(_angle, -angle_rise_factor, MAX_ANGLE)
        _torque = max(abs(_torque), -0.1*MAX_TORQUE) #match torque signal to angle
      #case range(9):
      #  _torque = float(key)
      #case '-':
      #  _torque = -1 * _torque
      #case _: # other key press
      #  pass
    print_cmd_state()
  except: # ignore other keys
    pass

def on_release(key):
  global _torque
  global _angle
  global _mode
  _torque = 0.0

  try:
    match key.char:
      case ['w', 'q'] if _mode == modes['TORQUE_CONTROL']:
        time.sleep(2) # hold torque for 2s  
        _torque = rise_and_decay(_torque, torque_decay_factor, MAX_TORQUE)
      #case ['a', 'd'] if _mode == modes['ANGLE_CONTROL']:
      #  pass
      #case _:
      #  pass  
    print_cmd_state()  
  except:
    pass 

def motor_tester(bus):
  panda = Panda()
  panda.set_can_speed_kbps(bus, motor_bus_speed)
  # Now set the panda from its default of SAFETY_SILENT (read only) to SAFETY_ALLOUTPUT
  print("Setting Panda to All Output mode...")
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  print("Enable all pandas busses...") #in case panda was sleeping
  panda.set_power_save(False) #enable all the busses
  print("Enable panda usb heartbeat..") #so it doesn't stop
  #_thread.start_new_thread(heartbeat_thread, (panda,))
  heartbeat = threading.Thread(target=heartbeat_thread, args=(panda,), daemon=True, name='panda heartbeat')
  heartbeat.start()

  #Request off control mode first - in case motmo
  # or was in SoftOff fault mode
  
  print("\nRequesting motor OFF mode...")
  dat = steering_msg_cmd_data(modes["OFF"], 0, 0.0, 0.0)
  print('Sent:' + ''.join('\\x{:02x}'.format(b) for b in dat)) ##b"\x30\x00\x00\x00\x00"
  panda.can_send(MSG_STEERING_COMMAND_FRAME_ID, dat, bus)
  
  global _torque
  global _angle
  global _mode

  _torque = 0.0
  _angle = 0.0
  _mode = modes['OFF']

  tx_t = threading.Thread(target=CAN_tx_thread, args=(panda, bus), daemon=True)
  rx_t = threading.Thread(target=CAN_rx_thread, args=(panda, bus), daemon=True)
  tx_t.start()
  rx_t.start()
  
  print(f"Mode: {[name for name, val in modes.items() if val == _mode][0]}")
  time.sleep(0.1)
  _mode = modes['TORQUE_CONTROL']
  print(f"Mode: {[name for name, val in modes.items() if val == _mode][0]}")
  print("\nEnter torque value or used W/S keys to increase/decrease torque and A/D angle. Q to quit:") #show this before CAN messages spam the terminal

  with keyboard.Listener(on_press=on_press, on_release=on_release, suppress=True) as listener:  # Setup the listener of keyboard
  #with keyboard.Listener(on_press=on_press, suppress=True) as listener:  # Setup the listener of keyboard
    try:
      listener.join()  # Join the thread to the main thread 
    except KeyboardInterrupt:
      return False
  
  print("Disabling output on Panda...")
  panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Simple motor tester-runner with numeric or game lile controls",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, help="CAN bus id to which motor is connected", default=2)
  args = parser.parse_args()
  print("Killing boardd to release USB...")
  try: #useful if run on EON
    subprocess.run(['pkill', '-f', 'boardd'])
  except:
    pass
  motor_tester(args.bus)
