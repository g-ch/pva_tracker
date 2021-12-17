#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import dynamic_reconfigure.client
import serial
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QWidget, QLabel, QMessageBox
import sys

class Q_Window(QWidget):
    def __init__(self):
        super(Q_Window, self).__init__()

        self.open_success = False
        self.init_success = False

        self.serial = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.5)  # /dev/ttyUSB0
        if self.serial.isOpen():
            print("open succeeded")
            self.open_success = True
        else:
            print("open failed. Will try again")

        self.client = dynamic_reconfigure.client.Client("ground_reconfigure_server")
        self.param_dic = self.client.get_configuration()

        self.initUI()

    def initUI(self):
        self.instruction = QLabel(u'请在调节完参数后点击下方按钮更新参数', self)
        self.instruction.move(120, 50)

        self.send_Button = QPushButton(u'发送', self)
        self.send_Button.clicked.connect(self.send)
        self.send_Button.resize(200, 50)
        self.send_Button.move(200, 120)
        self.send_Button.setEnabled(False)

        self.msg_label = QLabel(u'', self)
        self.msg_label.resize(200, 50)
        self.msg_label.move(200, 200)
        self.msg_label.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.init_timer_thread_operation)
        self.timer.start(500)

        self.setGeometry(500, 500, 600, 300)
        self.setWindowTitle('Ground PVA PID Parameters Reconfigure')
        self.show()

    def closeEvent(self, event):
        return_code = QMessageBox.information(self, u'注意', u'确定关闭程序吗?', QMessageBox.Yes | QMessageBox.No)
        if return_code == QMessageBox.No:
            event.ignore()
        else:
            event.accept()

    def init_timer_thread_operation(self):
        if not self.open_success:           ### Try to open serial
            self.msg_label.setText(u'连接中...')
            if self.serial.isOpen():
                print("open succeeded")
                self.open_success = True
            else:
                print("open failed")

        elif not self.init_success:   ### Try to synchronize parameters to those in onboard computer
            self.msg_label.setText(u'等待初始化参数...')
            self.timer.setInterval(10)
            data = self.serial.read_all()
            if data != "":
                print(data)
            if data == 'init':
                self.timer.stop()
                self.msg_label.setText(u'初始化参数接收中...')
                counter = 0
                param_dict_received = {}
                flag = 'name'
                name = ''
                check_flag = 0
                while counter < 200:
                    rospy.sleep(0.05)
                    counter += 1
                    recv = self.serial.read_all()
                    if recv == 'complete':
                        self.init_success = True
                        break
                    elif recv != '':
                        print("received data: " + recv)
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

                if self.init_success:
                    print('Initialize succeeded!')
                    self.msg_label.setText(u'初始化成功！')
                    self.send_Button.setEnabled(True)
                    self.serial.write(str(check_flag))   ## Send to check if initialization is successful
                    rospy.sleep(0.2)
                    self.client.update_configuration(param_dict_received)  # Update to initialized parameters

                else:
                    self.timer.start(500)

        else:
            self.timer.stop()

    def send(self):
        return_code = QMessageBox.information(self, u'注意', u'确定发送吗?', QMessageBox.Yes | QMessageBox.No)
        if return_code == QMessageBox.No:
            self.msg_label.setText(u'发送已取消')
            self.msg_label.repaint()
        else:
            self.send_Button.setEnabled(False)
            self.msg_label.setText(u'发送中...')
            self.msg_label.repaint()
            send_counter = 1
            while send_counter < 10:
                send_counter += 1
                param_dic_to_set = self.client.get_configuration()
                # Start flag
                data = "start"
                self.serial.write(data)
                rospy.sleep(0.1)
                check_flag = 0
                for k in param_dic_to_set.keys():
                    if k != 'groups':
                        self.serial.write(k)
                        print(k)
                        rospy.sleep(0.1)
                        self.serial.write(str(param_dic_to_set[k]))
                        check_flag += param_dic_to_set[k]
                        print(param_dic_to_set[k])
                        rospy.sleep(0.1)
                # End flag
                data = 'complete'
                self.serial.write(data)
                rospy.sleep(0.1)
                data = str(check_flag)
                self.serial.write(data)
                rospy.sleep(0.1)

                # Waiting for Feedback
                update_succeed = False
                for i in range(20):
                    data = self.serial.read_all()
                    try:
                        if data != '':
                            if abs(float(data) - check_flag) < 0.01:
                                self.msg_label.setText(u'参数更改成功！')
                                self.msg_label.repaint()
                                update_succeed = True
                                break
                            else:
                                self.msg_label.setText(u'校验失败！')
                                self.msg_label.repaint()

                    except Exception as e:
                        print(e)
                        break
                    rospy.sleep(0.1)

                if not update_succeed:
                    self.msg_label.setText(u'重试中...！')
                else:
                    break

        self.send_Button.setEnabled(True)


if __name__ == '__main__':
    rospy.init_node('ground_reconfigure', anonymous=True)

    app = QApplication(sys.argv)
    ex = Q_Window()
    sys.exit(app.exec_())
