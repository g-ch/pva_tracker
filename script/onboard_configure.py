#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
import serial


if __name__ == '__main__':
    rospy.init_node('onboard_reconfigure', anonymous=True)
    client = dynamic_reconfigure.client.Client("tracker")

    serial = serial.Serial('/dev/black_usb_ttl', 57600, timeout=0.5)  # /dev/ttyUSB0

    while not rospy.is_shutdown():
        if serial.isOpen():
            print("open succeeded")
            break
        else:
            print("open failed")
            rospy.sleep(1)

    ''' 1.  Send initialized Parameters First '''
    init_param_dic = client.get_configuration()
    print("Sending initialization data")
    received_check_flag = 0;
    while not rospy.is_shutdown():
        # Start flag
        data = "init"
        serial.write(data)
        rospy.sleep(0.1)

        check_flag = 0
        for k in init_param_dic.keys():
            if k != 'groups':
                serial.write(k)
                rospy.sleep(0.1)
                serial.write(str(init_param_dic[k]))
                check_flag += init_param_dic[k]
                rospy.sleep(0.1)
        # End flag
        data = 'complete'
        serial.write(data)
        rospy.sleep(0.1)

        receive_flag = False
        for i in range(20):
            try:
                data = serial.read_all()
                print(data)
                if abs(float(data) - check_flag) < 0.01:
                    receive_flag = True
                    received_check_flag = check_flag;
                    break
                else:
                    rospy.sleep(0.1)
            except Exception as e:
                print(e)
                break

        if receive_flag:
            break

        print("Initialization failed. Will try again...")

    print("Initialization succeeded! check_flag=" + str(received_check_flag))

    ''' 2. Now receive Updates From Ground Station '''
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        data = serial.read_all()
        ready_to_update = False

        if data == 'start':
            counter = 0
            param_dict_received = {}
            flag = 'name'
            name = ''
            check_flag = 0
            while counter < 100:
                rospy.sleep(0.05)
                counter += 1
                recv = serial.read_all()
                print(recv)
                if recv == 'complete':
                    rospy.sleep(0.1)
                    recv = serial.read_all()
                    print("check_flag received =" + recv)
                    print("check_flag calculated =" + str(check_flag))
                    try:
                        if abs(check_flag - float(recv)) < 0.01:
                            print("Update data received!")
                            ready_to_update = True
                        break
                    except Exception as e:
                        print(e)
                        break

                elif recv != '':
                    if flag == 'name':
                        name = recv
                        flag = 'data'
                    else:
                        try:
                            data = float(recv)
                            param_dict_received[name] = data
                            check_flag += data
                            flag = 'name'
                        except Exception as e:
                            print(e)
                            break

            if ready_to_update:
                serial.write(str(check_flag))  ## Send to check if initialization is successful
                rospy.sleep(0.2)
                client.update_configuration(param_dict_received)  # Update to initialized parameters
                print("Update Complete!")
            else:
                print("Update failed!")
