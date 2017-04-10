import sys
import Leap
import time
import math
import socket

host = ''
port = 12345
s = socket.socket()
s.bind((host, port))
s.listen(1)
connection,address = s.accept()

#Max and Min depending on the arm's range
MAX_Y = 300
MIN_Y = 120
MAX_Z = 200
MIN_Z = 1

#Lenght of the arm in mm
LENGTH1 = 120
LENGTH2 = 120
#Claw distance
minimum_claw_distance = 10
normalize = 3

square = lambda x:  x*x

def check_boundaries(stable_position):
    hand_stable_position = stable_position.to_float_array()
    if (stable_position[1] < MIN_Y):
        hand_stable_position[1] = MIN_Y
    if (stable_position[1] > MAX_Y):
        hand_stable_position[1] = MAX_Y
    if (stable_position[2] > MAX_Z):
        hand_stable_position[2] = MAX_Z
    return hand_stable_position

def find_distance(x1,y1,z1,x2,y2,z2):
    return math.sqrt(square(x2-x1) + square(y2-y1) + square(z2-z1))

def get_base_angle(x):
    n = 100 * normalize
    dummy = 1.5+2*x/n
    angle = 90 + math.cos(dummy)*90
    return angle

def get_other_angles(x,y,z):
    y = y*1.5 + 40
    z = -z*1.5
    try:
        theta1 = math.acos((square(z) + square(y) - square(LENGTH1) - square(LENGTH2)) / (2 * LENGTH1 * LENGTH2))
        theta2 = math.asin(((LENGTH1+LENGTH2*math.cos(theta1))*y-LENGTH2*math.sin(theta1)*z)/
                            (square(LENGTH1)+square(LENGTH2)+2*LENGTH1*LENGTH2*math.cos(theta1)))
        dummy_elbow_angle = (math.floor(45 + math.degrees(theta2))-45)*2
        dummy_shoulder_angle = (math.floor(180 - math.degrees(theta1))-45)*2
        elbow_angle = adjust_angle(dummy_elbow_angle,'elbow')
        shoulder_angle = adjust_angle(dummy_shoulder_angle,'shoulder')
    except:
        elbow_angle =  90
        shoulder_angle = 90
    return (shoulder_angle,elbow_angle)

def get_claw_angle(finger1,finger2):                            #10 is open, 70 is closed
    distance_fingers = find_distance(finger1[0], finger1[1], finger1[2], finger2[0], finger2[1], finger2[2])
    angle = 90 - distance_fingers
    if angle < 10:
        claw_angle = 10
    elif angle >= 70:
        claw_angle = 70
    else:
        claw_angle = angle
    return math.floor(claw_angle)

def adjust_angle(angle,joint):
    if (joint == 'elbow'):
        if angle > 90:
            result = 90.0
        elif angle < 0:
            result = 0.0
        else:
            result = angle
    
    elif (joint == 'shoulder'):
        if angle > 90:
            result = 90.0
        elif angle < 20:
            result = 20.0
        else:
            result = angle
    return result


class LeapMotionListener(Leap.Listener):
    state_names = ['STATE_INVALID','STATE_START','STATE_UPDATE','STATE_END']

    def on_init(self,controller):
        print "Initialized"
        self.clamp = 0

    def on_connect(self,controller):
        print "Connected"

    def on_disconnect(self,controller):
        print "Motion Sensor Disconnected"

    def on_exit(self,controller):
        print "Terminated"
        connection.send('quit')
        connection.close()

    def on_frame(self,controller):
        #Frame that gets sent from Leap motion to computer
        frame = controller.frame()
        data = connection.recv(1024)
        print(data)
        #data = 'a'
        if data == 'rover':
            print "here"
            frame = 0
        else:            
            if (len(frame.hands) != 0 and len(frame.hands) != 2):
                if not frame.hands.is_empty:
                    hand = frame.hands[0]
                    #Claw
                    finger1 = hand.pointables[0].tip_position
                    finger2 = hand.pointables[1].tip_position

                    # Pitch: Angle between the negative z axis and the projection of the vector onto y-z plane.
                    # in other words, rotation around x axis), if vector points upward -> 0<angle<pi, if downward -pi<angle<0
                    hand_pitch = hand.direction.pitch * Leap.RAD_TO_DEG
                    if hand_pitch < 0.0:
                        wrist_angle = 0
                    elif hand_pitch > 90:
                        wrist_angle =90.0
                    else:
                        wrist_angle = math.floor(hand_pitch) 

                    hand_roll = hand.direction.roll * Leap.RAD_TO_DEG
                    if hand_roll < 0.0:
                        rotation_angle = math.floor(hand_roll) * -1
                    elif hand_roll > 119:
                        rotation_angle =119.0
                    else:
                        rotation_angle = math.floor(hand_roll)

                    position = hand.stabilized_palm_position
                    dummy_vector = Leap.Vector(position[0],position[1],position[2])
                    hand_stable_position = check_boundaries(dummy_vector)
                    
                    #Arm angles
                    theta1,theta2 = get_other_angles(0,-10+hand_stable_position[1]/normalize,hand_stable_position[2]/normalize)
                    base_angle = math.floor(180 - get_base_angle(position[0]/1.5))/2
                    shoulder_angle = theta1

                    elbow_angle = theta2  
                    claw_angle = get_claw_angle(finger1,finger2) 

                    angles = str(int(math.floor(base_angle))+32) + ',' + \
                    str(int(shoulder_angle)+32) + ',' +  str(int(elbow_angle)+32) + ',' +\
                    str(int(wrist_angle)+32) + ',' + str(int(claw_angle)+32) + '\n'
                    print(angles)
                    connection.send(angles)
                    time.sleep(.15)
            
            else:
                connection.send('77,122,77,77,42')
                pass
def main():
    #print "Press enter to start"
    begin = raw_input("Press enter to start: ")
    listener = LeapMotionListener()
    print('listener')
    controller = Leap.Controller()
    controller.add_listener(listener)
    print "Press q to quit"
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        controller.remove_listener(listener)
        connection.close()
        pass
    finally:
        controller.remove_listener(listener)
        connection.close()
if __name__== "__main__":
    main()
