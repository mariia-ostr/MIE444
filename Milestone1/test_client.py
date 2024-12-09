import socket
import time
from datetime import datetime
import serial

# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def responseValue(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = item[1][1]
        print(out_string)

    return out_string
'''
def CollisionCheck(cmds: str, responses_list: list):
    
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    collision=True
    
    for item in zip(cmd_list, responses_list, valid):
        if item[1][1]!='True' or item[1][1]!='False' or item[1][1]!='Not Found':
            print(item[1][1])
            if float(item[1][1])<=2:
                collision=False
     
    return collision
'''
def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0




############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = False # If true, run this. If false, skip it
while RUN_COMMUNICATION_CLIENT:
    # Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    [responses, time_rx] = receive()
    if responses[0]:
        print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
    else:
        print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")




############## Main section for the open loop control algorithm ##############
# The sequence of commands to run
CMD_SEQUENCE = ['w0:36', 'r0:90', 'w0:36', 'r0:90', 'w0:12', 'r0:-90', 'w0:24', 'r0:-90', 'w0:6', 'r0:720']
LOOP_PAUSE_TIME = 0.5 # seconds


# Main loop
RUN_DEAD_RECKONING = True # If true, run this. If false, skip it
ct = 0
while RUN_DEAD_RECKONING:
    # Pause for a little while so as to not spam commands insanely fast
    time.sleep(LOOP_PAUSE_TIME)


    # If the command sequence hasn't been completed yet
    ##if ct < len(CMD_SEQUENCE):

        # Check an ultrasonic sensor 'u0'
    packet_tx = packetize('u0')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
            ir0=responseValue('u0',responses)
            if '.'in ir0:
                try:
                    ir0=float(responseValue('u0',responses))
                except ValueError:
                    pass
                
        # Check an ultrasonic sensor 'u1'
    packet_tx = packetize('u1')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
            ir1=responseValue('u1',responses)
            if '.'in ir1:
                try:
                    ir1=float(responseValue('u1',responses))
                except ValueError:
                    pass
                
            
    packet_tx = packetize('u2')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Ultrasonic 2 reading: {response_string('u2',responses)}")
            ir2=float(responseValue('u2',responses))
                
                
            
    packet_tx = packetize('u3')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Ultrasonic 3 reading: {response_string('u3',responses)}")
            ir3=float(responseValue('u3',responses))
            
    packet_tx = packetize('u4')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Ultrasonic 4 reading: {response_string('u4',responses)}")
            ir4=float(responseValue('u4',responses))
    packet_tx = packetize('u5')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Ultrasonic 5 reading: {response_string('u5',responses)}")
            ir5=float(responseValue('u5',responses))
            
        #obstacle avoidance 
        
    list_sensor=[ir3,ir4,ir5]
    max_sensor=list_sensor[0]
         
    for i in range(len(list_sensor)):
            if max_sensor<list_sensor[i]:
                max_sensor=list_sensor[i]
    print(max_sensor)
                
    if ir5 < 2.5 :
        transmit(packetize('w0:-1.5'))  # Move backward if obstacle is too close
    if ir1 < 2.5: #back sensor too close 
        transmit(packetize('w0:1.5'))
    if (ir3+ir4+ir5)/3 < 4:#corner 
        
        transmit(packetize('r0:180'))
    if ir4 < 2:  # Obstacle on the right, turn left
        transmit(packetize('w0:-0.5'))
        transmit(packetize('r0:-10'))
        #transmit(packetize('w0:0.5'))
    if ir3 < 2:  # Obstacle on the left, turn right
        transmit(packetize('w0:-0.5'))
        transmit(packetize('r0:10'))
        #transmit(packetize('w0:0.5'))
    if ir4>3 and ir2<0.5: #right hand sensor too close 
        transmit(packetize('w0:1.5'))
        transmit(packetize('r0:-30'))
    if ir3>3 and ir0<0.5: #left hand sensor 
        transmit(packetize('w0:1.5'))
        transmit(packetize('r0:30'))
    
        
        
        

# Logic based on the sensor with the greatest value
# If front sensor has the maximum value and side sensors are clear
    if max_sensor == ir5:
        if ir4 < 2:  # Obstacle on the right, turn left
            transmit(packetize('w0:-0.5'))
            transmit(packetize('r0:-8'))
            transmit(packetize('w0:0.8'))
        if ir3 < 2:  # Obstacle on the left, turn right
            transmit(packetize('w0:-0.5'))
            transmit(packetize('r0:8'))
            transmit(packetize('w0:0.8'))
        if ir3 > 2.5 and ir4 > 2.5:
            transmit(packetize('w0:2'))  # Move forward if both sides are clear
        

# If the right-side sensor has the maximum value, turn right and move forward
    elif max_sensor == ir4:
        
        if ir3<1:
           transmit(packetize('w0:-0.5'))
           transmit(packetize('r0:-15'))
        
        else:
            transmit(packetize('r0:25'))
            transmit(packetize('w0:2'))

# If the left-side sensor has the maximum value, turn left and move forward
    elif max_sensor == ir3:
        
        if ir4<2:
           transmit(packetize('w0:-0.5'))
           transmit(packetize('r0:15'))
        
        else:
            transmit(packetize('r0:-25'))
            transmit(packetize('w0:2'))

    transmit(packetize('w0:0.7'))
    
        #check front sensor only  
    ''' if float(responseValue('u0',responses))<=3 and responseValue('u0',responses)!='':  
            transmit(packetize('w0:-0.5')) 
        #check front and right sensor 
        if float(responseValue('u0',responses))<=3 and float(responseValue('u1',responses))<=2: 
            transmit(packetize('w0:-0.5')) 
            transmit(packetize('r0:-20'))
        #check front and left sensor       
        if float(responseValue('u0',responses))<=3 and float(responseValue('u2',responses))<=2: 
            transmit(packetize('w0:-0.5')) 
            transmit(packetize('r0:20'))   
            '''            
            

        # Check the remaining three sensors: gyroscope, compass, and IR
    packet_tx = packetize('g0,c0,i0')
    if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Other sensor readings:\n{response_string('g0,c0,i0',responses)}")
            
            '''
         # Send a drive command
        if RUN_DEAD_RECKONING is True:
            packet_tx = packetize(CMD_SEQUENCE[ct])
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                print(f"Drive command response: {response_string(CMD_SEQUENCE[ct],responses)}")
              '''  
       

        # If we receive a drive response indicating the command was accepted,
        # move to the next command in the sequence
        
        #if responses[0]:
    if responses[0][1] == 'True':
               ct += 1
                
    # If the command sequence is complete, finish the program
    ##RUN_DEAD_RECKONING = False
           # print("Sequence complete!")
        
