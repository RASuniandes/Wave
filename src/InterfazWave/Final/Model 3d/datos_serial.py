import csv
import serial_comm as my_serial

import csv
import queue
import re
import threading
from time import perf_counter
import PySimpleGUI as sg
import serial.tools.list_ports  # Para listar los puertos COM disponibles
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np
import os

class Application:

    def __init__(self, *args, **kwargs):
        super(Application, self).__init__(*args, **kwargs)
        baud_rate = 115200
        gui_queue = queue.Queue()
        serial_connector = my_serial.SerialObj(baud_rate)

        headerFont = ('Helvetica', 16)
        middleFont = ('Helvetica', 14)
        contextFont = ('Helvetica', 12)
        smallFont = ('Helvetica', 10)
        
        self.ax=0
        self.ay=0
        self.az=0
        self.yaw=0
        self.pitch=0
        self.roll=0
        self.init_time=perf_counter()
        self.current_time=0
        
        self.number_of_rows=9
        
        
                # Datos para la animación
        self.ax_data = []
        self.ay_data = []
        self.az_data = []
        self.yaw_data = []
        self.pitch_data = []
        self.roll_data = []
        
        self.yaw_data1 = []
        self.pitch_data1 = []
        self.roll_data1 = []
        
        self.time_data = []

        # Configuración de la interfaz de usuario PySimpleGUI
        # Aquí debes agregar tu código de la interfaz de usuario
        
        # Iniciar hilo para animación
        
        sg.theme('DarkBlue')

        layout = [[sg.Text('GET ACCELEROMETER GYROSCOPE\nSAMPLING DATA VIA SERIAL', font=headerFont)],
                  [sg.Text('Select your serial port', font=contextFont),
                   sg.Button('Serial Port Reload', size=(20, 1), font=smallFont)],
                  [sg.Listbox(values=[x[0] for x in my_serial.SerialObj.get_ports()],
                              size=(40, 6),
                              key='_SERIAL_PORT_LIST_',
                              font=contextFont,
                              enable_events=True)],
                  [sg.Text('', key='_SERIAL_PORT_CONFIRM_', size=(40, 1), font=middleFont, ), ],
                  [sg.Text('Buad Rate: {} bps'.format(baud_rate), size=(40, 1), font=middleFont, ), ],
                  [sg.Text('How many samples?', font=contextFont, ), sg.VerticalSeparator(),
                   sg.Input(do_not_clear=True, enable_events=True, key='_SAMPLE_IN_', font=contextFont, )],
                 [sg.Canvas(key='-CANVAS-')],
                  [sg.HorizontalSeparator()],
                  [sg.Text('Serial Comm Status', font=contextFont, pad=((6, 0), (20, 0))), ],
                  [sg.Text('', key='_OUTPUT_', size=(40, 2), font=middleFont, ), ],
                  [sg.Button('Start', key='_ACT_BUTTON_', font=middleFont, size=(40, 1), pad=((0, 0), (0, 0)))],
                  [sg.Button('Exit', font=middleFont, size=(40, 1), pad=((0, 0), (20, 0)))],
                  [sg.Text('ThatProject - Version: 0.1', justification='right', size=(50, 1), font=smallFont, ), ]]

        self.window = sg.Window('Simple Serial Application', layout, size=(320, 440), keep_on_top=True)
        self.canvas_elem = self.window['-CANVAS-']
        self.canvas = self.canvas_elem.TKCanvas
        
        
                # Configurar la gráfica de Matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-180, 180)
        self.lines = [self.ax.plot([], [], lw=2)[0] for _ in range(6)]

        # Dibujar la figura de Matplotlib en el Canvas de PySimpleGUI
        self.fig_agg = self.draw_figure(self.canvas, self.fig)
        while True:
            event, values = self.window.Read(timeout=100)

            if event is None or event == 'Exit':
                break

            if event == 'Serial Port Reload':
                self.get_ports()

            if event == '_SERIAL_PORT_LIST_':
                self.window['_SERIAL_PORT_CONFIRM_'].update(value=self.window['_SERIAL_PORT_LIST_'].get()[0])

            if event == '_SAMPLE_IN_' and values['_SAMPLE_IN_'] and values['_SAMPLE_IN_'][-1] not in ('0123456789'):
                self.window['_SAMPLE_IN_'].update(values['_SAMPLE_IN_'][:-1])

            if event == '_ACT_BUTTON_':
                print(self.window[event].get_text())
                if self.window[event].get_text() == 'Start':

                    if len(self.window['_SERIAL_PORT_LIST_'].get()) == 0:
                        self.popup_dialog('Serial Port is not selected yet!', 'Serial Port', contextFont)

                    elif len(self.window['_SAMPLE_IN_'].get()) == 0:
                        self.popup_dialog('Set Sampling Count', 'Sampling Number Error', contextFont)

                    else:
                        self.stop_thread_trigger = False
                        self.thread_serial = threading.Thread(target=self.start_serial_comm,
                                                              args=(serial_connector,
                                                                    self.window[
                                                                        '_SERIAL_PORT_LIST_'].get()[
                                                                        0],
                                                                    int(self.window[
                                                                            '_SAMPLE_IN_'].get()),
                                                                    gui_queue, lambda: self.stop_thread_trigger),
                                                              daemon=True)
                        self.thread_serial.start()
                        self.window['_ACT_BUTTON_'].update('Stop')
                        
                        
                        

                else:
                    self.stop_thread_trigger = True
                    self.thread_serial.join()
                    self.window['_ACT_BUTTON_'].update('Start')

            try:
                message = gui_queue.get_nowait()
            except queue.Empty:
                message = None
            if message is not None:
                self.window['_OUTPUT_'].Update(message)
                if 'Done' in message:
                    self.window['_ACT_BUTTON_'].update('Start')
                    self.popup_dialog(message, 'Success', contextFont)

        self.window.Close()
    def draw_figure(self,canvas, figure):
        figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
        figure_canvas_agg.draw()
        figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
        return figure_canvas_agg   
    def run_animation(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-180, 180)
        self.lines = []
        for _ in range(6):  # Yaw, Pitch, Roll, ax, ay, az
            line, = self.ax.plot([], [], lw=2)
            self.lines.append(line)
        self.ani = FuncAnimation(self.fig, self.update_animation, blit=True)
        plt.show()

    def update_animation(self, frame):
        # Actualizar datos del gráfico
        if self.time_data:
            self.ax.set_xlim(self.time_data[0], self.time_data[-1])
        for i, line in enumerate(self.lines):
            line.set_data(self.time_data, [self.yaw_data, self.pitch_data, self.roll_data, self.ax_data, self.ay_data, self.az_data][i])
        return self.lines
    
    
    def update_data(self, current_time, ax, ay, az, yaw, pitch, roll,yaw1,pitch1,roll1,):
        # Actualizar las listas de datos para la animación
        self.time_data.append(current_time)
        self.ax_data.append(ax)
        self.ay_data.append(ay)
        self.az_data.append(az)
        self.yaw_data.append(yaw)
        self.pitch_data.append(pitch)
        self.roll_data.append(roll)
        self.yaw_data1.append(yaw1)
        self.pitch_data1.append(pitch1)
        self.roll_data1.append(roll1)
        
        # Recortar las listas para mantener un tamaño manejable
        if len(self.time_data) > 1000:  # Ajusta este número según sea necesario
            self.time_data.pop(0)
            self.ax_data.pop(0)
            self.ay_data.pop(0)
            self.az_data.pop(0)
            self.yaw_data.pop(0)
            self.pitch_data.pop(0)
            self.roll_data.pop(0)
            self.yaw_data1.pop(0)
            self.pitch_data1.pop(0)
            self.roll_data1.pop(0)

    def popup_dialog(self, contents, title, font):
        sg.Popup(contents, title=title, keep_on_top=True, font=font)

    def get_ports(self):
        self.window['_SERIAL_PORT_LIST_'].Update(values=[x[0] for x in my_serial.SerialObj.get_ports()])

    def start_serial_comm(self, serial_connector, serialport, sample_num, gui_queue, stop_thread_trigger):

        start_time = 0

        serial_connector.connect(serialport)
        if serial_connector.is_connect():

            gui_queue.put('Serial Connected!!')

            n = 0
            while n < sample_num:

                try:
                    if stop_thread_trigger():
                        break

                    data = serial_connector.get_data()
                    if data is not None:

                        if n == 0:
                            gui_queue.put(' - Data Transmitting ::: Wait! ')
                            start_time = perf_counter()

                        decode_string = data.decode('utf-8')
                        print(decode_string)
                        if len(decode_string.split(',')) == self.number_of_rows:
                            self.ax,self.ay,self.az,self.yaw,self.pitch,self.roll,self.yaw1,self.pitch1,self.roll1 = map(float, decode_string.split(','))
                            self.current_time=perf_counter()-self.init_time
                            
                            
                            self.update_data(self.current_time, self.ax, self.ay, self.az, self.yaw, self.pitch, self.roll,self.yaw1,self.pitch1,self.roll1)
                            n += 1
                            percent = n / sample_num * 100
                            
                            current_folder = os.getcwd()  # Obtiene la ruta de la carpeta actual
                            filename = os.path.join(current_folder, "datos.csv")  # Define el nombre y ruta del archivo
                            self.csv_writer(filename, n, decode_string)

                            if percent % 10 == 0:
                                gui_queue.put('Saving to CSV File: {}% complete'.format(int(percent)))

                except OSError as error:
                    print(error)

                except UnicodeDecodeError as error:
                    print(error)

        serial_connector.disconnect()
        time_taken = (perf_counter() - start_time)
        
        sampling_rate = sample_num / time_taken
        gui_queue.put('Sampling Rate: {} hz ::: Done!'.format(int(sampling_rate)))
        return

    def csv_writer(self, filename, index, data):
        headers = ['time', 'ax', 'ay', 'az', 'yaw', 'pitch', 'roll', 'yaw1', 'pitch1', 'roll1']
        file_exists = os.path.isfile(filename)
        with open(filename, 'a') as f:
            writer = csv.writer(f, delimiter=",", quoting=csv.QUOTE_NONE, escapechar=' ')
            if not file_exists:
                writer.writerow(headers)  # Escribir encabezados si el archivo es nuevo
            writer.writerow([index, re.sub(r"\s+", "", data), 0, 0,
                             0])  # Dummy data for magnetometers, it doesn't use magnetometer in matlab.


if __name__ == '__main__':
    Application()