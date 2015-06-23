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
 
    def __init__(self,pub):
        super(Settings, self).__init__()
        #set all default values
        self.pub = pub

        # values of settings that are published
        # always numbers, will be changed
        self.settings = {'RLim': 95.0,      
                            'LLim': 85.0,
                            'heading': 0,
                            'NBins': 100,
                            'range': 2}

        # limits against which user input is checked
        # can be reference in settingsDict or fixed value
        # limitsDict is never changed
        self.limits = { 'RLimMin': 'LLim',    
                            'RLimMax': 360,
                            'LLimMin': 0,
                            'LLimMax': 'RLim',
                            'headingMin':0,
                            'headingMax':360,
                            'NBinsMin':0,
                            'NBinsMax':100,
                            'rangeMin':0,
                            'rangeMax':5
                            }

        # Text that will be displayed to explain which value is changed
        self.text = { 'RLim': 'RLim: ',
                      'LLim': 'LLim: ',
                      'heading': 'heading: ',
                      'NBins': 'NBins: ',
                      'range': 'range: ',
                      }

        


    def initUI(self):
        self.grid = QtGui.QGridLayout()
        self.setLayout(self.grid)
        #self.statusbar = self.statusBar()
        
        self.DefineLayout()
        self.setWindowTitle('Sonar Settings')
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.Publish)
        self.timer.start(1000)
    
        self.show()

    def Publish(self):
        """
        Publish sonar settings regularly
        """
        self.pub.publish(RLim = self.settings['RLim'], 
                         LLim = self.settings['LLim'], 
                         Heading = self.settings['heading'], 
                         NBins = self.settings['NBins'], 
                         Range = self.settings['range'])



    def DefineLayout(self):
        """
        Define what is shown where in the graphical interface
        """
        self.label_RLim = QtGui.QLabel(self.UpdateText('RLim'))
        self.grid.addWidget(self.label_RLim, 1, 0)

        self.input_RLim = QtGui.QLineEdit()
        self.grid.addWidget(self.input_RLim, 1, 1)

        self.button = QtGui.QPushButton("Update Settings")
        self.grid.addWidget(self.button, 2, 0)
        self.button.clicked.connect(self.buttonClicked)

    def buttonClicked(self):
        try:
            self.settings['RLim'] = float(self.input_RLim.text())
            self.label_RLim.setText(self.UpdateText('RLim'))

            # publish new values
            self.Publish()

            
        except:
            print("Please check your input!")

    def UpdateText(self, setting):
        text = self.text[setting] + str(self.settings[setting]) + "\n"
        text = text + "Enter new value between " + str(self.limits[setting + 'Min']) + " and " + str(self.limits[setting + 'Max'])
        return text
        

def sonarSettingTalker():
    pub = rospy.Publisher('sonar_updateSettings', sonar_setting, queue_size=10)
    rospy.init_node('sonar_updateSettings', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    app = QtGui.QApplication([])
    settings = Settings(pub)
    settings.initUI()   # use graphical interface
    sys.exit(app.exec_())
   
if __name__ == '__main__':
    sonarSettingTalker()
