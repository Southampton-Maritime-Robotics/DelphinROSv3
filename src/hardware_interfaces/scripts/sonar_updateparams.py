#!/usr/bin/python
import rospy
import sys
import serial
import time
import numpy
from std_msgs.msg import String
from PyQt4 import QtCore, QtGui


# Import required data types
from hardware_interfaces.msg import sonar
from hardware_interfaces.msg import sonar_setting

""" Interface for updating various sonar parameters
"""


"""
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing sonar parameters %s" % rospy.get_time())
        pub.publish(RLim, LLim, heading, NBins, Range)
        rate.sleep()

"""


class Settings(QtGui.QWidget):
 
    def __init__(self):
        super(Settings, self).__init__()
        #set all default values
        self.RLim = 95.00
        self.LLim = 85.00
        self.heading = 0
        self.NBins = 100
        self.Range = 2

        self.initUI()

    def initUI(self):
        self.grid = QtGui.QGridLayout()
        self.setLayout(self.grid)
        #self.statusbar = self.statusBar()
        
        self.DefineLayout()
        self.setWindowTitle('Sonar Settings')
        self.show()

    def DefineLayout(self):
        self.text_RLim = "The current maximum angle of the sonar head sector scan is: "
        self.label_RLim = QtGui.QLabel(self.UpdateText(self.text_RLim, self.RLim, self.LLim, 360.0))
        self.grid.addWidget(self.label_RLim, 1, 0)

        self.input_RLim = QtGui.QLineEdit()
        self.grid.addWidget(self.input_RLim, 1, 1)

        self.grid.addWidget(QtGui.QPushButton("Update Settings"), 2, 0)

    def UpdateText(self, text, current_value, minLimit, maxLimit):
        # TODO make this use a class of settings
        text = text + str(current_value) + "\n" + "Enter new value between " + str(minLimit) + " and " + str(maxLimit)
        return text
        

def sonarSettingTalker():
    pub = rospy.Publisher('sonar_updateSettings', sonar_setting, queue_size=10)
    rospy.init_node('sonar_updateSettings', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    app = QtGui.QApplication([])
    settings = Settings()
    sys.exit(app.exec_())
   
if __name__ == '__main__':
    sonarSettingTalker()
