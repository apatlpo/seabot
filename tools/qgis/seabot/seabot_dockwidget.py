# -*- coding: utf-8 -*-°
"""
/***************************************************************************
 SeabotDockWidget
                                 A QGIS plugin
 Seabot
 Generated by Plugin Builder: http://g-sherman.github.io/Qgis-Plugin-Builder/
                             -------------------
        begin                : 2018-10-31
        git sha              : $Format:%H$
        copyright            : (C) 2018 by Thomas Le Mézo
        email                : thomas.le_mezo@ensta-bretagne.org
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
"""

import os, time

from PyQt5 import QtGui, QtWidgets, uic
from PyQt5.QtCore import pyqtSignal, QTimer, QFile, QFileInfo
from PyQt5.QtCore import QDate, QTime, QDateTime, Qt
from PyQt5.QtWidgets import QApplication, QWidget, QInputDialog, QLineEdit, QFileDialog, QTreeWidgetItem
from PyQt5.QtGui import QIcon

from seabot.src.seabotLayerLivePosition import SeabotLayerLivePosition
from seabot.src.boatLayerLivePosition import BoatLayerLivePosition
from seabot.src.seabotMission import *
from seabot.src.missionLayer import *
from seabot.src.seabotIridiumIMAP import *

FORM_CLASS, _ = uic.loadUiType(os.path.join(
    os.path.dirname(__file__), 'seabot_dockwidget_base.ui'))

class SeabotDockWidget(QtWidgets.QDockWidget, FORM_CLASS):

    closingPlugin = pyqtSignal()
    timer_seabot = QTimer()
    timer_boat = QTimer()
    timer_mission = QTimer()
    timer_IMAP = QTimer()
    flag = False
    count = 0

    momsn_min = 0
    momsn_max = 0
    momsn_current = 0
    data_log = {}

    layerLivePosition = SeabotLayerLivePosition()
    boatLivePosition = BoatLayerLivePosition()
    seabotMission = SeabotMission()
    missionLayer = MissionLayer()
    dataBaseConnection = DataBaseConnection()
    imapServer = ImapServer()

    def __init__(self, parent=None):
        """Constructor."""
        super(SeabotDockWidget, self).__init__(parent)
        # Set up the user interface from Designer.
        # After setupUI you can access any designer object by doing
        # self.<objectname>, and you can use autoconnect slots - see
        # http://qt-project.org/doc/qt-4.8/designer-using-a-ui-file.html
        # #widgets-and-dialogs-with-auto-connect

        # print(self.__dir__)
        self.setupUi(self)

        ### Timer handle
        self.timer_seabot.timeout.connect(self.process_seabot)
        self.timer_seabot.setInterval(5000)

        self.timer_boat.timeout.connect(self.process_boat)
        self.timer_boat.setInterval(1000)

        self.timer_mission.timeout.connect(self.process_mission)
        self.timer_mission.setInterval(1000)

        self.timer_IMAP.timeout.connect(self.process_IMAP)
        self.timer_IMAP.setInterval(1000)

        ### UI pushButton handle
        # Init tree Widget
        self.treeWidget_iridium.setColumnCount(2)
        self.tree_log_data = self.treeWidget_iridium.setHeaderLabels(["Parameter","Data"])

        # Config tab
        self.pushButton_seabot.clicked.connect(self.enable_timer_seabot)
        self.pushButton_boat.clicked.connect(self.enable_timer_boat)

        self.spinBox_gnss_trace.valueChanged.connect(self.update_vanish_trace)

        self.pushButton_server_save.clicked.connect(self.server_save)
        self.pushButton_server_delete.clicked.connect(self.server_delete)
        self.comboBox_config_email.currentIndexChanged.connect(self.select_server)
        self.pushButton_server_connect.clicked.connect(self.server_connect)

        # Mission tab
        self.pushButton_open_mission.clicked.connect(self.open_mission)
        self.pushButton_mission.clicked.connect(self.enable_timer_mission)

        # State tab
        self.pushButton_state_rename.clicked.connect(self.rename_robot)
        self.pushButton_state_previous.clicked.connect(self.previous_log_state)
        self.pushButton_state_next.clicked.connect(self.next_log_state)
        self.comboBox_state_imei.currentIndexChanged.connect(self.update_state_imei)

        # Fill list of email account
        self.update_server_list()

        self.update_robots_list()
        self.update_state_imei()

    def server_save(self, event):
        email = self.lineEdit_email.text()
        password = self.lineEdit_password.text()
        server_ip = self.lineEdit_server_ip.text()
        server_port = self.lineEdit_server_port.text()
        t_zero = self.dateTimeEdit_last_sync.dateTime().toString(Qt.ISODate)
        self.dataBaseConnection.save_server(email, password, server_ip, server_port, t_zero)
        self.update_server_list()
        return True

    def server_delete(self, event):
        id_config = self.comboBox_config_email.currentData()
        self.dataBaseConnection.delete_server(id_config)
        self.update_server_list()
        return True

    def update_server_list(self):
        self.comboBox_config_email.clear()
        email_list = self.dataBaseConnection.get_email_list()
        for email in email_list:
            self.comboBox_config_email.addItem(str(email["config_id"]) + " - " + email["email"], email["config_id"])

    def update_robots_list(self, index_comboBox=-1):
        self.comboBox_state_imei.clear()
        robot_list = self.dataBaseConnection.get_robot_list()
        if(len(robot_list)==0):
            return

        for robot in robot_list:
            if robot["name"] != None:
                self.comboBox_state_imei.addItem(robot["name"] + " (" + str(robot["imei"]) + ")", robot["imei"])
            else:
                self.comboBox_state_imei.addItem(str(robot["imei"]), robot["imei"])
        
        if index_comboBox==-1:
            self.comboBox_state_imei.setCurrentIndex(len(robot_list)-1)
        else:
            self.comboBox_state_imei.setCurrentIndex(index_comboBox)

    def rename_robot(self):
        if(self.comboBox_state_imei.currentIndex() != -1):
            currentIndex = self.comboBox_state_imei.currentIndex()
            text, ok = QInputDialog().getText(self, "Database update",
                                         "Robot name:", QLineEdit.Normal,
                                         self.dataBaseConnection.get_robot_name(self.comboBox_state_imei.currentData()))
            if ok and text:
                self.dataBaseConnection.update_robot_name(text, self.comboBox_state_imei.currentData())
                self.update_robots_list(currentIndex)

    def select_server(self, index):
        if index != -1:
            server_id = self.comboBox_config_email.currentData()
            server_data = self.dataBaseConnection.get_server_data(server_id)
            self.lineEdit_email.setText(str(server_data["email"]))
            self.lineEdit_password.setText(str(server_data["password"]))
            self.lineEdit_server_ip.setText(str(server_data["server_ip"]))
            self.lineEdit_server_port.setText(str(server_data["server_port"]))
            self.dateTimeEdit_last_sync.setDateTime(server_data["last_sync"])

    def open_mission(self, event):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","Mission Files (*.xml);;All Files (*)", options=options)
        if fileName!="":
            print("filename=", fileName)
            self.seabotMission.load_mission_xml(fileName)

            file_info = QFileInfo(fileName)
            self.label_mission_file.setText(file_info.fileName())
            self.missionLayer.update_mission_layer(self.seabotMission)

    def closeEvent(self, event):
        self.timer_seabot.stop()
        self.timer_boat.stop()
        self.timer_IMAP.stop()
        self.timer_mission.stop()

        self.imapServer.stop_server()

        self.pushButton_seabot.setChecked(False)
        self.closingPlugin.emit()
        event.accept()

    def set_enable_form_connect(self, enable):
        if(enable):
            self.comboBox_config_email.setEnabled(True)
            self.lineEdit_email.setEnabled(True)
            self.lineEdit_password.setEnabled(True)
            self.lineEdit_server_ip.setEnabled(True)
            self.lineEdit_server_port.setEnabled(True)
            self.pushButton_server_save.setEnabled(True)
            self.pushButton_server_delete.setEnabled(True)
            self.dateTimeEdit_last_sync.setEnabled(True)
        else:
            self.comboBox_config_email.setEnabled(False)
            self.lineEdit_email.setEnabled(False)
            self.lineEdit_password.setEnabled(False)
            self.lineEdit_server_ip.setEnabled(False)
            self.lineEdit_server_port.setEnabled(False)
            self.pushButton_server_save.setEnabled(False)
            self.pushButton_server_delete.setEnabled(False)
            self.dateTimeEdit_last_sync.setEnabled(False)

    def add_item_treeWidget(self, val1, val2=None, nb_digit=-1):
        item = None
        if(val2==None):
            text = self.data_log[val1]
        else:
            text = val2

        if nb_digit>0:
            text = round(float(text), nb_digit)
        elif nb_digit==0:
            text = int(round(float(text)))

        item = QTreeWidgetItem([str(val1), str(text)])   
        self.treeWidget_iridium.addTopLevelItem(item)

    def fill_treeWidget_log_state(self):
        self.treeWidget_iridium.clear()
        self.add_item_treeWidget("message_id")
        self.add_item_treeWidget("ts", datetime.datetime.fromtimestamp(self.data_log["ts"]))
        self.add_item_treeWidget("east", nb_digit=0)
        self.add_item_treeWidget("north", nb_digit=0)
        self.add_item_treeWidget("gnss_speed", nb_digit=2)
        self.add_item_treeWidget("gnss_heading", nb_digit=0)
        self.add_item_treeWidget("safety_published_frequency", nb_digit=0)
        self.add_item_treeWidget("safety_depth_limit", nb_digit=0)
        self.add_item_treeWidget("safety_batteries_limit", nb_digit=0)
        self.add_item_treeWidget("safety_depressurization", nb_digit=0)
        self.add_item_treeWidget("enable_mission", nb_digit=0)
        self.add_item_treeWidget("enable_depth", nb_digit=0)
        self.add_item_treeWidget("enable_engine", nb_digit=0)
        self.add_item_treeWidget("enable_flash", nb_digit=0)
        self.add_item_treeWidget("battery0", nb_digit=2)
        self.add_item_treeWidget("battery1", nb_digit=2)
        self.add_item_treeWidget("battery2", nb_digit=2)
        self.add_item_treeWidget("battery3", nb_digit=2)
        self.add_item_treeWidget("pressure", nb_digit=0)
        self.add_item_treeWidget("temperature", nb_digit=1)
        self.add_item_treeWidget("humidity", nb_digit=2)
        self.add_item_treeWidget("waypoint", nb_digit=0)
        self.add_item_treeWidget("last_cmd_received")

    def update_state_info(self):
        # Get current momsn
        self.momsn_current = self.dataBaseConnection.get_momsn_from_message_id(self.data_log["message_id"])

        # Update Text
        self.label_state_info.setText(str(self.momsn_current) + "/ [" + str(self.momsn_min) + ", " + str(self.momsn_max) + "]")

    def update_momsn_bounds(self):
        self.momsn_min, self.momsn_max = self.dataBaseConnection.get_bounds_momsn(self.comboBox_state_imei.currentData())

    def update_vanish_trace(self, value):
        if(value==-1):
            self.boatLivePosition.set_nb_points_max(value, False)
        else:
            self.boatLivePosition.set_nb_points_max(value, True)


    ###########################################################################
    ### Handler Button

    def enable_timer_seabot(self):
        if(self.pushButton_seabot.isChecked()):
            self.timer_seabot.start()
        else:
            self.timer_seabot.stop()

    def enable_timer_boat(self):
        if(self.pushButton_boat.isChecked()):
            self.boatLivePosition.start()
            self.timer_boat.start()
        else:
            self.timer_boat.stop()
            self.boatLivePosition.stop()

    def enable_timer_mission(self):
        if(self.pushButton_mission.isChecked()):
            # self.boatLivePosition.start()
            self.timer_mission.start()
        else:
            self.timer_mission.stop()
            # self.boatLivePosition.stop()

    def server_connect(self):
        self.pushButton_server_connect.setStyleSheet("background-color: red")
        if(self.pushButton_server_connect.isChecked()):
            self.set_enable_form_connect(False)
            self.imapServer.set_server_id(self.comboBox_config_email.currentData())
            
            ## Thread IMAP
            self.imapServer.start_server()
            
            ## UI update
            self.timer_IMAP.start()
        else:

            self.set_enable_form_connect(True)
            self.label_server_log.setText("Disconnected")
            ## Thread IMAP
            self.imapServer.stop_server()
            self.timer_IMAP.stop()

    def next_log_state(self):
        data = self.dataBaseConnection.get_next_log_state(self.data_log["message_id"])
        if(data != None):
            self.data_log = data
            self.update_state_info()
            self.fill_treeWidget_log_state()

    def previous_log_state(self):
        data = self.dataBaseConnection.get_previous_log_state(self.data_log["message_id"])
        if(data != None):
            self.data_log = data
            self.update_state_info()
            self.fill_treeWidget_log_state()

    def update_state_imei(self):
        if(self.comboBox_state_imei.currentIndex() != -1):
            self.data_log, self.momsn_current = self.dataBaseConnection.get_last_log_state(self.comboBox_state_imei.currentData())
            
            self.fill_treeWidget_log_state()
            self.update_momsn_bounds()
            self.update_state_info()

    ###########################################################################
    ## TIMERS processing

    def process_seabot(self):
        if(self.layerLivePosition.update()):
            self.label_status.setText("Connected - " + str(self.count))
            self.count += 1
        else:
            self.label_status.setText("Error")
            self.count = 0

    def process_boat(self):
        self.boatLivePosition.update()

    def process_IMAP(self):
        self.label_server_log.setText(self.imapServer.get_log())
        if(self.imapServer.get_is_connected()):
            self.pushButton_server_connect.setStyleSheet("background-color: green")
        else:
            self.pushButton_server_connect.setStyleSheet("background-color: red")

    
    def process_mission(self):
        # Update mission set point on map
        self.missionLayer.update_mission_set_point(self.seabotMission)

        # Update IHM with mission data set point
        wp = self.seabotMission.get_current_wp()
        # print(wp)
        if(wp!=None):
            if(wp.get_depth()==0.0):
                self.label_mission_status.setText("SURFACE")
            else:
                self.label_mission_status.setText("UNDERWATER")

            self.label_mission_start_time.setText(str(wp.get_time_start()))
            self.label_mission_end_time.setText(str(wp.get_time_end()))
            self.label_mission_depth.setText(str(wp.get_depth()))
            self.label_mission_waypoint_id.setText(str(wp.get_id())+"/"+str(self.seabotMission.get_nb_wp()))
            self.label_mission_time_remain.setText(str(wp.get_time_end()-datetime.datetime.now()))

            wp_next = self.seabotMission.get_next_wp()
            if(wp_next != None):
                self.label_mission_next_depth.setText(str(wp_next.get_depth()))
            else:
                self.label_mission_next_depth.setText("-")
        else:
            self.label_mission_status.setText("NO WAYPOINTS")
            self.label_mission_waypoint_id.setText(str(self.seabotMission.get_current_wp_id()+1) + "/"+str(self.seabotMission.get_nb_wp()))
