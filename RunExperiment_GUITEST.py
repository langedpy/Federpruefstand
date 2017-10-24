from PyQt5 import QtGui
from PyQt5.QtCore import *
import sys
import h5py
import numpy as np
#from PyDAQmx import *
import time
import pyqtgraph as pg
from os import path
import GUI_Federpruefstand as gui
#import Eurotherm2408 as ET

class Spring_app(QtGui.QMainWindow, gui.Ui_MainWindow):
    def __init__(self, parent=None):
        super(Spring_app, self).__init__(parent)    #super function: access variables, methos etc. in gui file.
        self.setupUi(self)  #This is defined in gui file. Sets up Gui layout
        #Variablen declaration
        self.savecounter = 100
        self.counter = 0


        #Initialization of the Worker Thread to run the experiment simultanously to the GUI
        self.exp_obj = Worker_experiment()     #  create Worker and Thread inside the Form
        self.exp_thread = QThread()
        self.exp_obj.moveToThread(self.exp_thread)  #  Move the Worker object to the Thread object
        self.exp_thread.started.connect(self.exp_obj.startup_experiment) # Connect Thread started signal to Worker operational slot method
        self.exp_obj.finished.connect(self.exp_thread.quit)
        self.exp_obj.dataReady.connect(self.onDataReady)

        #Following are the GUI event connections
        self.pushButton_3.clicked.connect(self.save_settings)
        self.pushButton.clicked.connect(self.start_experiment)
        self.pushButton_2.clicked.connect(self.stop_experiment)

    def save_settings(self):
        """
        Saves the user set experiment variables to global variables and shows them in the experiment overview labels.

        """
        global t_max, t_min, h_rate, m_rate, max_cycles, save_path
        try
            t_max = int(self.t_maxLineEdit.text())
            t_min = int(self.t__minLineEdit.text())
            h_rate = int(self.heatingRateLineEdit.text())
            m_rate = int(self.measureRateLineEdit.text())
            max_cycles = int(self.maximalCyclesLineEdit.text())
            #savepath
            self.label_setTmax.setText(str(t_max))
            self.label_setTmin.setText(str(t_min))
            self.label_setheatingrate.setText(str(h_rate))
            #self.label_setmeasurerate.setText(str(m_rate))
        except
            msg_save_settings = QMessageBox()
            msg_save_settings.setIcon(QMessageBox.Warning)
            msg_save_settings.setText("Please check the parameter values again!")
            msg.setStandardButtons(QMessageBox.Ok)

    @pyqtSlot()
    def start_experiment(self):
        """
        Starts the experiment, by setting the according parameter for run_experiment in the Worker Thread to True. Starts the Worker Thread. Additionally the start_time is set here.
        :return:
        """
        start_time = time.time()
        print("experiment started")
        self.exp_obj.isStillGoodToRun = True
        self.exp_thread.start()

    @pyqtSlot()
    def stop_experiment(self):
        """
        Stops the experiment, by setting the according parameter for run_experiment in the Worker Thread to false.
        :return:
        """
        print("experiment stopped")
        self.exp_obj.isStillGoodToRun = False

    @pyqtSlot()
    def onDataReady(self, actual_d, actual_d_time, actual_t_time, actual_t):
        """
         Is called, when the Worker Thread has data ready. appends the collected actual data to the specific arrays. Calles save_data, when self.savecounter datapoints have been accumulated. Then all arrays and the counter is reset to zero.
        :param actual_d: The parameters represent the last data grabbed by the sensors.
        :param actual_d_time:
        :param actual_t_time:
        :param actual_t:
        :return:
        """
        #distance = np.append(distance, actual_d)
        #dist_t = np.append((dist_t, actual_d_time))
        temperature = np.append(temperature, actual_t)
        temp_t = np.append(temp_t, actual_t_time)
        self.counter += 1
        if self.counter == self.savecounter:
            self.save_data(distance, dist_t, temperature, temp_t)
            temperature = numpy.zeros((0,), dtype=numpy.float64)
            temp_t = numpy.zeros((0,), dtype=numpy.float64)
            distance = numpy.zeros((0,), dtype=numpy.float64)
            dist_t = numpy.zeros((0,), dtype=numpy.float64)
            self.counter = 0

    @pyqtSlot()
    def save_data(self, distance, dist_t, temperature, temp_t):
        """
        Saves the last collected data (amount specified by self.savecounter) to a hdf5 file, which can be opened in Origin Pro.
        :param distance: The Parameters are the accumulated arrays from onDataReady.
        :param dist_t:
        :param temperature:
        :param temp_t:
        :return: -
        """
            folder = h5py.File('measurements/Testfile.hdf5', 'a')
            grp_temp = folder.create_group("/temperature")
            data_temp = grp_temp.create_dataset('temperature measurement', (0,), dtype=[('time_temp', '<f8'), ('temperature', '<f8')], maxshape=(None,))
            data_temp.resize(data_temp.shape[0] + temperature.shape[0], axis=0)
            data_temp['temperature', self.savecounter:] = temperature
            data_temp['time_temp', self.savecounter:] = temp_t

            #grp_dist = folder.create_group("/distance")
            #data_dist = grp_dist.create_dataset('distance measurement', (, ),
            #           dtype = [('time_dist', '<f8'), ('distance', '<f8')], maxshape = (None,))
            #data_dist.resize(data_dist.shape[0] + distance.shape[0], axis=0)
            #data_temp['distance', self.savecounter:] = distance
            #data_temp['time_dist', self.savecounter:] = dist_t
            folder.close()



class Worker_experiment(QObject):
    finished = pyqtSignal()
    dataReady = pyqtSignal(float, float, float, float)

    @pyqtSlot()
    def startup_experiment(self):
        """
        Initializes the Experiment: Connection to the sensors
        :return: Starts the run_experiment function, which is recalled by a QTimer.
        """
        #self.heatercontroller = Eurotherm2408(COM8, 1)
        self.isHeatingokay = True
        self.run_experiment()

    @pyqtSlot()
    def run_experiment(self):
        """
        Collecting all sensor data and is rerun by a QTimer with the measurement rate. Controls the temperature via if conditions and increasing the eurotherms setpoint according to heating rate.
        :return: Emits a signal to the Main Thread to grab the data, when it is collected.
        """
        if self.isStillGoodToRun == True:
            try:
                print("bla")
                #self.actual_d = read_distance()
                #self.actual_d_time = time.time()
                self.actual_t = self.heatercontroller.getpv()
                self.actual_t_time = time.time()
                self.dataReady.emit(self.actual_d, self.actual_d_time, self.actual_t_time, self.actual_t)

                if self.isHeatingokay == True:
                    if self.actual_t < t_max:
                        self.heatercontroller.setpv(self.actual_t + h_rate / 1)
                       # self.heatercontroller.setpv(self.actual_t+h_rate/m_rate)
                    else:
                        self.isHeatingokay = False
                        self.heatercontroller.setpv(self.actual_t - h_rate/1)
                        # self.heatercontroller.setpv(self.actual_t - h_rate/m_rate)
                else:
                    if self.actual_t >= t_max:
                        self.heatercontroller.setpv(self.actual_t - h_rate / 1)
                       # self.heatercontroller.setpv(self.actual_t - h_rate/m_rate)
                    else:
                        self.isHeatingokay = True
                        self.heatercontroller.setpv(self.actual_t + h_rate/1)
                        # self.heatercontroller.setpv(self.actual_t+h_rate/m_rate)
            finally:
                #QTimer.singleShot(1000/m_rate, self.run_experiment)
                QTimer.singleShot(1000, self.run_experiment)

    def read_distance(self, NIport):
        """
        Reads out Values of the NI Card, in this case the distance.
        :param NIport: The correct NI Cards port has to be specified.
        :return: Returns the data value for the distance (float between 0.0 and 10.0), which has to be calibrated later on.
        """
        # Declaration of variable passed by reference
        self.taskHandle = TaskHandle()
        self.read = int32()
        self.data = numpy.zeros((1,), dtype=numpy.float64)

        try:
            # DAQmx Configure Code
            DAQmxCreateTask("", byref(taskHandle))
            DAQmxCreateAIVoltageChan(taskHandle, "Dev1/%s" % port, "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts,
                                     None)  # umgestellt auf RSE
            DAQmxCfgSampClkTiming(taskHandle, "", 10000.0, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, 1000)

            # DAQmx Start Code
            DAQmxStartTask(taskHandle)

            # DAQmx Read Code
            DAQmxReadAnalogF64(taskHandle, 1, 10.0, DAQmx_Val_GroupByChannel, data, 1000, byref(read), None)

        except DAQError as err:
            print("DAQmx Error: %s") % err
        finally:
            if self.taskHandle:
                # DAQmx Stop Code
                DAQmxStopTask(taskHandle)
                DAQmxClearTask(taskHandle)
        return (self.data)




def main():
    app = QtGui.QApplication(sys.argv)
    form=Spring_app()
    form.show()
    app.exec()

if __name__ == '__main__':
    main()