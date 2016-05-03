import rospy
import sys
import traceback
import serial
import time
import numpy
from std_msgs.msg import String
from PyQt4 import QtCore, QtGui


# Import required data types
from hardware_interfaces.msg import sonar
from hardware_interfaces.msg import sonar_setting


class Settings(QtGui.QWidget):
    """ 
    Graphical interface for manually updating 
    various sonar parameters

    gets input from graphical interface,
    regularly publishes the values set in the interface
    """


 
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
        self.text = { 'RLim': 'RLim (default: 95.0): ',
                      'LLim': 'LLim (default: 85.0): ',
                      'heading': 'heading (default: 3200): ',
                      'NBins': 'NBins (default: 200): ',
                      'range': 'range (default: 20): ',
                      }

         # Edit lines where new values can be entered
        self.editLines = {}
        for key in self.settings:
            self.editLines[key] = ''

        # dict of QLabels, so they can be automatically updated
        self.QLabels = {}
        for key in self.settings:
            self.QLabels[key] = QtGui.QLabel("")

      


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
        d = 0
        for key in self.settings:
            d +=1
            self.QLabels[key].setText(self.UpdateText(key))
            self.grid.addWidget(self.QLabels[key], d, 0)
            self.editLines[key] = QtGui.QLineEdit()
            self.grid.addWidget(self.editLines[key], d, 1)

        self.button = QtGui.QPushButton("Update Settings")
        self.grid.addWidget(self.button, d+1, 0)
        self.button.clicked.connect(self.buttonClicked)

    def buttonClicked(self):
        try:
            for key in self.settings:
                # TODO: before accepting values, checks using the limits dictionary
                # should be done!
                # TODO: make the lines display the current value instead of being empy
                self.settings[key] = float(self.editLines[key].text())
                self.QLabels[key].setText(self.UpdateText(key))

            # publish new values
            self.Publish()

            
        except:
            print("Please check your input!")
            traceback.print_exc(file=sys.stdout)

    def UpdateText(self, setting):
        text = self.text[setting] + str(self.settings[setting]) + "\n"
        text = text + "Enter new value between " + str(self.limits[setting + 'Min']) + " and " + str(self.limits[setting + 'Max'])
        return text
        

def sonarSettingTalker():
    pub = rospy.Publisher('sonar_updateSettings', sonar_setting, queue_size=10)
    rospy.init_node('sonar_updateSettings', anonymous=True)
 
    app = QtGui.QApplication([])
    settings = Settings(pub)
    settings.initUI()   # use graphical interface
    sys.exit(app.exec_())
 
