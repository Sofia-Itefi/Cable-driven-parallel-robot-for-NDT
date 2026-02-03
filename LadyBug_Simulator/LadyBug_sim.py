'''Developed by Sofia Aparicio
TOMOGRAPHY
VERSION DE DASH 2.13.0 instalarla con pip install dash==2.13.0
Last changes made on Oct 22, 2025. Madrid Spain
Manual and Automatic Sonic Inspection Program
The program allows robot movement and hammering both manually and automatically, after defining a grid.
Two Arduinos are used: an Arduino UNO for the motors (it controls 2 motors) and an Arduino UNO WiFi R4 to activate the fans and the hammering wirelessly.

The automatic program reads the grid from the cc_start_grid.csv file, where the dimensions are in millimeters. If microstepping is used, the program updates the cc_start_grid.csv file, saving the last adjusted coordinate.

A new grid can be created and saved using the Download CSV button, and then the file can be renamed to cc_start_grid.csv to use the new grid.
You can also load the grid from a previously saved file.

From the dashboard, you can reset the fans (it sends the message "5" to the Arduino WiFi) and change the fan speed (it sends the message "4 speed" to the Arduino WiFi).

In this version for the La Vela tower, the reels were replaced and two cables were used.
***********************************************'''
#############################################
#######################
####################################################################
#########################LIBRERIA###################################
####################################################################
####################################################################

import base64
import json
import re
import time
import serial
from serial import *
from time import sleep
from operator import itemgetter
import csv

import dash_bootstrap_components as dbc
import dash_daq as daq
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import scipy.interpolate as sc
from dash import Dash, dcc, html, ctx, no_update
from dash.dependencies import Input, Output, State
from dash.exceptions import PreventUpdate

#Para que no salga el mensaje de POST /_dash-update-component HTTP/1.1 message y que lea correctamente al pausar el programa
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
import datetime
import math
import socket
##################################################################################################################
########################################
import time
import queue

class FakeSerial:
    def __init__(self, timeout=2):
        self.timeout = timeout
        self.is_open = True
        self._rx_queue = queue.Queue()

    def write(self, data):
        cmd = data.decode().strip()
        print(f"[SIM] Received command: {cmd}")

        # Simula retardo de procesamiento
        time.sleep(0.05)

        # Respuesta básica tipo GRBL
        if cmd.startswith(("G0", "G1", "G28", "$")):
            self._rx_queue.put("ok\n")
        else:
            self._rx_queue.put("error: unsupported command\n")

    def readline(self):
        try:
            return self._rx_queue.get(timeout=self.timeout).encode()
        except queue.Empty:
            return b""

    def flushInput(self):
        pass

    def close(self):
        self.is_open = False
####################################################################
USE_FAKE = True

class ArduinoInterface:
    def send(self, message: str):
        raise NotImplementedError

class FakeArduino(ArduinoInterface):
    def send(self, message: str):
        # Simula latencia de red / procesamiento
        time.sleep(0.02)

        return {
            "ack": "OK",
            "received": message,
            "source": "FAKE_UDP"
        }

class UDPArduino(ArduinoInterface):
    def __init__(self, ip, port=5005, timeout=1.0):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def send(self, message: str):
        try:
            self.sock.sendto(bytes(message, "utf-8"), self.addr)
            data, address = self.sock.recvfrom(2048)
            return json.loads(data.decode('utf-8'))
        except Exception:
            return {"ack": "NO_DEVICE"}

####################################################################
#########################DEFINICIÓN VARIABLES#######################
####################################################################
####################################################################

#####################MOTORS#######################################################################################
#Puerto Arduino Motores
portname = 'COM8'#'COM3'
SIMULATION=1

if SIMULATION:
    s = FakeSerial()
else:
    s = serial.Serial(portname, 115200) #.Serial
print("Connected correctly.\nStarting up CABLE ROBOT..")

# Define steps per revolution for each motor
steps_per_revolution = [1.55, 1.49]#Nuevo Carrete Torre de la VELA

#Posición de los motores
w_cnc = 16#8.50#Nuevo Carrete Torre de la VELA# 5.0#2.34#Ancho en m
h_cnc = 7.3#8.19#2.0#Alto en m

# Define motor (pulley) positions for R1, R2
motor_positions = [
    (0, h_cnc),   # R1 (Top-Left)
    (w_cnc, h_cnc)  # R2 (Top-Right)
]
#Velocidad de movimiento (parece que no funciona por eso lo hago a capón con $110 $111...
movespeed = " F750"
probespeed = " F750"

#####Micropasos en mm
cm_to_move=0.05# cada 5 cm 0.02# Se pone en m. 2 cm cada vez.#0.005 #para 0.5 cm cada vez que se le da un mini paso

#Número de cuerdas (3 en cada eje)
n_cuerdas=3#3#Si ponemos 4 cuerdas (2 cuerdas en cada eje), multiplicamos por 2.JJ

###################################################################################################################

############################VENTILADORES Y SOLENOIDE###############################################################
##Para conectarse por WIFI a un Arduino con el ventilador
host = '127.0.0.1'#'192.168.10.158'
port = 50008
##################################################################################################################

############################################OTRAS Variables########################################################
stop_press=0
pause_press=0

#Valores de x e y del grid por defecto
value_grid_x=round(w_cnc,1)
value_grid_y=round(h_cnc,1)

styles = {
    'pre': {
        'border': 'thin lightgrey solid',
        'overflowX': 'scroll'
    }
}


#Valores iniciales del grid
numero_puntos_hor_ini=27
numero_puntos_ver_ini=8
value_grid_x_ini=16#value_grid_x
value_grid_y_ini=3.5#value_grid_y
x_offset_gpr_ini=1.5
y_offset_gpr_ini=0
################################################################################################################

####################################################################
####################################################################
############################DASHBOARD###############################
####################################################################
####################################################################

###########################Función que crea todo el dashboard######################################
app = Dash(__name__, external_stylesheets=[dbc.themes.ZEPHYR, dbc.icons.BOOTSTRAP])

app.layout = html.Div([
    html.Div(children=[
        html.H1('Meshgrid SONIC'),
        
        html.Div(children=[
            html.Label('Number of HORIZONTAL points'),
            dbc.Input(id='slider-input-mesh-horizontal', type='number', value=numero_puntos_hor_ini, min=0, max=50, step=1),
            html.Div(id='slider-output-mesh-horizontal'),

            html.Label('Number of VERTICAL points    '),
            dbc.Input(id='slider-input-mesh-vertical', type='number', value=numero_puntos_ver_ini, min=0, max=50, step=1),
            html.Div(id='slider-output-mesh-vertical'),
        ]),

        html.Div(children=[
            html.Label('Enter width (X) [m] of the work area:'),
            dbc.Input(id='x-offset', type='number', value=value_grid_x_ini, min=0, max=w_cnc, step=0.01),
            html.Div(id='output-x-offset'),

            html.Label('Enter height (Y) [m] of the work area:'),
            dbc.Input(id='y-offset', type='number', value=value_grid_y_ini, min=0, max=h_cnc, step=0.01),
            html.Div(id='output-y-offset')
        ]),

        html.Div(children=[            
            html.Label('Enter offset width (X) [m] of the work area:'),
            dbc.Input(id='x-offset-gpr', type='number', value=x_offset_gpr_ini, min=0, max=w_cnc, step=0.1),
            html.Div(id='output-x-offset-gpr'),

            html.Label('Enter offset height (Y) [m] of the work area:'),
            dbc.Input(id='y-offset-gpr', type='number', value=y_offset_gpr_ini, min=0, max=h_cnc, step=0.1),
            html.Div(id='output-y-offset-gpr')
        ]),

        # Memoria compartida para figura
        dcc.Store(id='figura-store'),

        dcc.Graph(id="plot"),
 
        dbc.Row([

            dbc.Col([
                dbc.Button('Generate start grid file', id='btn-nclicks-9', n_clicks=0, color="primary", outline=False),
                html.Div(id='start-grid-output')], width="auto")], justify='center'),
        dbc.Row([
            dbc.Col([
                dbc.Button('Plot grid from file', id='reload-btn', n_clicks=0, color="primary", outline=False),
                html.Div(id='update_plot_fromfile')], width="auto")], justify='center'),

        html.Div([
            dcc.Markdown("""
                ***Current Position:***
            """),
            html.Pre(id='click-data', style=styles['pre']),
        ], className='three columns'),

        dbc.Row([
            dbc.Col([
                dbc.Button([html.I(className="bi bi-download"), "  Download CSV"], id='btn-nclicks-12', color='success',
                           outline=True),
                html.Div(id='download-button'), html.Div(id='download-button-output'),
                dcc.Download(id="download-dataframe-xlsx")], width='auto'
            ),
            dbc.Col([
                dbc.Button('Delete previous datapoints', id='btn-nclicks-13', color='danger', outline=True),
                html.Div(id='delete-button-output'),
            ]),
            dbc.Col([
                dbc.Button([html.I(className="bi bi-arrow-counterclockwise"), "  UNDO"], id='btn-nclicks-123',
                           color='warning', outline=True),
                html.Div(id='undo-button-output'),
            ])
        ], justify='center')
    ], style={'padding': 50, 'flex': 1}),

    html.Div(children=[
        html.H1('Solenoid'),
        dbc.Row([dbc.Col([
            daq.Knob(
                id="frequency-knob",
                label="Frequency [Hz]",
                value=3,
                color={"gradient": True, "ranges": {"green": [0, 3], "yellow": [3, 7.5], "red": [7.5, 10]}}
            )]
        ),
            dbc.Col(
                daq.Knob(
                    id='hitting-knob',
                    label="Number of Hits [-]",
                    max=1000,
                    value=10
                )

            )]),
        dbc.Row([
            dbc.Col(html.Div(id='slider-output-frequency'), width="auto"),
            dbc.Col(html.Div(id='hitter-output-frequency'), width="auto")
        ], justify="center"),


        html.Br(),
        dbc.Row([
            dbc.Col([
                dbc.Button('Activate Fan', id='btn-nclicks-1', n_clicks=0, color='primary', outline=True),
                html.Div(id='position-solenoid-button')], width="auto"
            ),
            dbc.Col([
                dbc.Button('Deactivate Fan', id='btn-nclicks-7', n_clicks=0, color="primary", outline=True),
                html.Div(id='solenoid-backward-button')], width="auto"),
            dbc.Col([
                dbc.Button('Reset Fan', id='btn-nclicks-71', n_clicks=0, color="primary", outline=True),
                html.Div(id='reset-fan-button')], width="auto"),
            dbc.Col([
                html.Label('Enter fan velocity:'),
                dbc.Input(id='fan-velocity', type='number', value=28, min=0, max=35, step=1),
                html.Div(id='fan-velocity-output')], width="auto"),

            dbc.Col([
                dbc.Button('Send fan velocity', id='btn-nclicks-72', n_clicks=0, color="primary", outline=True),
                html.Div(id='send-fan-velocity-button'),
                html.Div(id='send-fan-velocity-output')], width="auto"),
            
            dbc.Col([
                dbc.Button([html.I(className="bi bi-exclamation-triangle"), "  SET ORIGIN"], id='btn-nclicks-200',
                           n_clicks=0, color="danger", outline=False),
                html.Div(id='set-zero-button'),
                html.Div(id='set-zero-output')], width="auto"),
            dbc.Col([
                dbc.Button('Start Hitting', id='btn-nclicks-2', n_clicks=0, color="success", outline=True),
                html.Div(id='start-solenoid-button'),
                html.Div(id='start-solenoid-output')], width="auto"),
        ], justify='center'),

        html.Br(),

        html.H1('Automatic Hitting'),

        html.Br(),
        dbc.Row([
            html.Div(children=[
                html.Label('Enter initial column to be inspected:'),
                dbc.Input(id='column-ini', type='number', value=0, min=0, max=200, step=1),
                html.Div(id='output-column-ini'),

                html.Label('Enter final column to be inspected:'),
                dbc.Input(id='column-fin', type='number', value=4, min=0, max=200, step=1),
                html.Div(id='output-column-fin')
            ]),

            dbc.Col([
                dbc.Button('Automatic Hitting', id='btn-nclicks-3', n_clicks=0, color="primary", outline=False),
                html.Div(id='start-automatic-output')], width="auto"),
            dbc.Col([
                dbc.Button('Stop Hitting', id='btn-nclicks-5', n_clicks=0, color="danger", outline=False),
                html.Div(id='stop-button')], width="auto"),
            dbc.Col([
                dbc.Button('Pause Hitting', id='btn-nclicks-6', n_clicks=0, color="danger", outline=False),
                html.Div(id='pause-button')], width="auto"),
         ], justify='center'),

        html.Br(),
        html.H1('Controller'),

        html.Br(),
        dbc.Row([

            dbc.Col([
                dbc.Button('X micro-adjustments [5cm steps] ', id='btn-nclicks-20', n_clicks=0, color="primary", outline=False),
                html.Div(id='x-micro')], width=5),
            dbc.Col([
                dbc.Button('-X micro-adjustments [5cm steps]', id='btn-nclicks-21', n_clicks=0, color="primary", outline=False),
                html.Div(id='-x-micro')], width=5),
            dbc.Col([
                dbc.Button('Y micro-adjustments [5cm steps]', id='btn-nclicks-22', n_clicks=0, color="primary", outline=False),
                html.Div(id='y-micro')], width=5),
            dbc.Col([
                dbc.Button('-Y micro-adjustments [5cm steps]', id='btn-nclicks-23', n_clicks=0, color="primary", outline=False),
                html.Div(id='-y-micro')], width=5),
            dbc.Col([
                dbc.Button('Save coordinates', id='btn-nclicks-40', n_clicks=0, color="success", outline=False),
                html.Div(id='save-coordinates')], width=5),

            ]
            , justify='center'
        
        ),
        html.Br(),

        dbc.Row([
            html.Img(src="/assets/ladybug_original2.png", style={'width': '600px'})

        ], justify='center')

    ], style={'padding': 50, 'flex': 1}),  # right side
], style={'display': 'flex', 'flex-direction': 'row'})
#################################################################################################################

####################################################################
####################################################################
############################FUNCIONES###############################
####################################################################
####################################################################

####################Función que envía los comandos GRBL escrito en el fichero grbl.gcode al arduino con los motores################
def stream():
    f = open('dynamic_text_files/grbl.gcode', 'r')
    log = open('dynamic_text_files/logfile.txt', 'a')
    # Wake up grbl
    temp = "\r\n\r\n"
    s.write(temp.encode())
    time.sleep(0.01)  # Wait for grbl to initialize
    s.flushInput()  # Flush startup text in serial input

    # Stream g-code to grbl
    for line in f:
        l = line.strip()  # Strip all EOL characters for consistency
        print('Sending: ' + l)
        tempo = l + '\n'
        s.write(tempo.encode())  # Send g-code block to grbl
        grbl_out = s.readline()  # Wait for grbl response with carriage return
        print(' : ' + str(grbl_out.strip()))
        log.write('Sending: ' + l + '\n')
        log.write(str(grbl_out.strip()) + '\n')

    # # Wait here until grbl is finished to close serial port and file.
    # input("  Press <ENTER> to exit and disable the control firmware.")

    # Close file and serial port
    f.close()
    log.close()
    # s.close()
    return "streamed"
####################################################################################################################################

#################################Función que guarda los puntos buenos ejecutados en el fichero probing_points.csv###################
#Función nueva para guardar los datos de los puntos dónde hemos golpeado en el CSV cuando se hace manualmente
def df_maker_nuevo():
    temp_lst = []
    # string to search in file
    word = 'PUNTO'#'PRB:'
    word1 = 'G90 X'
    word2 = 'G92 X'
    with open(r'dynamic_text_files/logfile.txt', 'r') as fp:
        # read all lines in a list
        line_numbers = []
        lines = fp.readlines()
        i=1
        fp.close()
        for line in lines:
            
            # check if string present on a current line
            if line.find(word) != -1:
                print(i)
                print(line)
                print(lines[i-2])
                if lines[i-2].find(word1)!= -1 or lines[i-2].find(word2)!= -1:
                    xy= [float(s)*1000 for s in re.findall(r'[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?', lines[i-2])]#Multiplicamos por 1000 para guardarlo en mm como el cc_start_grid.csv
                    temp_lst.append(xy[-2:])
                    line_numbers.append(i-1)
                elif lines[i-4].find(word1)!= -1 or lines[i-4].find(word2)!= -1:
                    xy= [float(s)*1000 for s in re.findall(r'[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?', lines[i-4])]#Multiplicamos por 1000 para guardarlo en mm como el cc_start_grid.csv
                    temp_lst.append(xy[-2:])
                    line_numbers.append(i-1)
            i=i+1
    df = pd.DataFrame(temp_lst, columns=['x', 'y'])
    # print(df)
    df.to_csv('probing_points.csv', sep='\t')
    print("*********Guardado CSV*********")
    print(line_numbers)
    return df, line_numbers
####################################################################################################################################

######################Función que busca las últimas coordenadas x e y dónde se ha movido el robot con mini move para guardarlas en el fichero star_grid############################################################
##Guardamos el valor de X e Y no de las longitudes de las cuerdas
def last_mini_move_movement_searcher():
    templst = []
    word = 'DISPLAY'
    word1 = 'MINIMOVE'
    with open(r'dynamic_text_files/logfile.txt', 'r') as fp:
        lines = fp.readlines()
        fp.close()

        for i in range(len(lines) - 1, -1, -1):
            if word in lines[i]:
                ultima_linea = i + 1  # (opcional) línea en formato 1-indexed
                print("ultima_linea "+ str(ultima_linea))
                numbers = re.findall(r'\d+', lines[i])
                print(numbers)
                break  # ← como queremos la última, salimos
            
    with open(r'dynamic_text_files/logfile.txt', 'r', encoding='utf-8') as fp:
        for i, linea in enumerate(fp):
            if i < ultima_linea:
                continue  # saltar hasta la línea deseada
            print("linea "+ linea)
            if word1 in linea:
                xy = [float(s)*1000 for s in re.findall(r'[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?', linea)]
                templst.append(xy)
                print(templst)

        fp.close()  

    if len(templst) == 0:
        templst = [[0.0, 0.0, 0.0]]
    else:
        templst = templst
        #Sustituye el punto del display por el punto actualizado con los microsteps
        #Remove last movement and put the new position
        X, Y= templst[-1]
        print(X)
        print(Y)
        f_coord = open("Coordinates_Input/cc_start_grid.csv", "r")
        data = f_coord.readlines()
        num_point=int(numbers[0])
        data[num_point+1]= str(num_point) + ','+ str(X)+','+ str(Y) + '\n'
        f_coord = open("Coordinates_Input/cc_start_grid.csv", "w")
        f_coord.writelines(data)
        f_coord.close()
    print(templst)
    return templst
###################################################################################################################################################################



#############################Función que busca las últimas coordenadas x e y dónde se ha movido el robot############################################################
##Guardamos el valor de X e Y no de las longitudes de las cuerdas
def last_movement_searcher():
    templst = []
    word = 'G90 X'
    word1 = 'G92 X'
    with open(r'dynamic_text_files/logfile.txt', 'r') as fp:
        lines = fp.readlines()
        fp.close()
        for line in lines:
            if line.find(word)!= -1 or line.find(word1)!= -1:
                xy = [float(s) for s in re.findall(r'[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?', line)]
                templst.append(xy)
    if len(templst) == 0:
        templst = [[0.0, 0.0, 0.0]]
    else:
        templst = templst
    print(templst)
    return templst
###################################################################################################################################################################

#############################Función que pasa de x e y a las longitudes de las cuerdas por Pitágoras#########################################
def calculate_rope_length(motor_pos, new_x, new_y):
    """ Calculate the rope length from the motor position to the new x and y coordinates. """
    motor_x, motor_y = motor_pos  # Motor's fixed position
    # Calculate Euclidean distance (rope length)
    return (math.sqrt((new_x- motor_x)**2 + (new_y - motor_y)**2))*n_cuerdas
#############################################################################################################################################

#########################Función que ajusta los pasos del motor con la longitud que se tiene que mover el robot##############################
def calculate_motor_steps(rope_length_change, steps_per_rev):
    """ Calculate the number of motor steps needed for the rope length change. """
    # Number of steps for the given rope length change
    print("calculate_motor_steps")
    print(rope_length_change)
    print(steps_per_rev)
    return (rope_length_change*steps_per_rev)*10
#############################################################################################################################################

##########Función que envía por WIFI al arduino UNO R4 WIFI un mensaje para activar/desactivar los ventiladores y el golpeo############
#### 0 Activa ventiladores y solenoide para golpeo. MESSAGE='0 freq nhits'
#### 1 Desactiva los ventiladores. MESSAGE='1'
#### 2 Activa ventiladores. MESSAGE='1'
#### 3 Activa solenoide para golpeo.MESSAGE='3 freq nhits'
def send_arduinoSolenoid(MESSAGE,host,port):
    data_recv=1
    arduino = FakeArduino() if USE_FAKE else UDPArduino(host)
    response = arduino.send(MESSAGE)
    if response.get("ack") == "OK":
        print("Arduino acknowledged command")
        data_recv=0
##    ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
##    ServerSocket.sendto(bytes(MESSAGE, "utf-8"), (host,port))
##    endTime = datetime.datetime.now() + datetime.timedelta(minutes=0.3)
##    while True:
##        if datetime.datetime.now() >= endTime:
##            print("*********No ha recibido el mensaje el Arduino*********")
##            break
##        else:
##            data, address = ServerSocket.recvfrom(2048)
##            data_str = data.decode('utf-8')
##            try:
##                # print(f"data : {data_str}, address : {address}")
##                print(f'data received: {data_str} {datetime.datetime.now()}')
##                data_recv=0
##                break
##            except:
##                print(f"invalid value, data : {data_str}, address : {address}")
##            if not data:
##                print("no data")
##    ServerSocket.close()
    return data_recv

#############################################################################################################################################

#####################Función para mover el robot al punto x e y######################################
def mini_move(x, y):  # TODO: add button input + code rest of function
    print("********************************************")
    print("*********Start movement*********************")
    print("********************************************")
    current_x = float(last_movement_searcher()[-1][-2])
    current_y = float(last_movement_searcher()[-1][-1])
    print(current_x, current_y)
    X = current_x + x
    Y = current_y + y
    print(X, Y)
    log = open('dynamic_text_files/logfile.txt', 'a')
    
    # Calculate new rope lengths for all motors
    new_rope_lengths = [calculate_rope_length(motor_pos, X, Y) for motor_pos in motor_positions]
    print("New Rope Lengths:", new_rope_lengths)
    # Calculate the number of steps each motor needs to move
    motor_steps = [calculate_motor_steps(change, steps_per_revolution[i]) for i, change in enumerate(new_rope_lengths)]
    print("Motor Steps Required:", motor_steps)
    command = "G90 X" + str(round(motor_steps[0],3)) + " Y" + str(round(motor_steps[1],3)) + movespeed + "\n"
    print(command)
    s.write(command.encode())
    command_x_y="G90 X" + str(round(X,3)) + " Y" + str(round(Y,3))+ "\n"
    l = command_x_y.strip()  # Strip all EOL characters for consistency
    log.write('Sending: ' + l + '\n')
    log.write('MINIMOVE: X' + str(X) + " Y" + str(Y)+'\n')
    log.close()
    mesg = "*********Arrived at {}, {} [m]*********".format(X, Y)
    print(mesg)
    return X,Y
#####################################################################################################    

#####################Función para calcular el número de filas o columnas de un fichero######################################
def contar_hasta_disminucion(datos, umbral):
    contador = 0
    for i in range(len(datos) - 1):
        actual = datos[i]
        siguiente = datos[i + 1]
        diferencia = abs(siguiente - actual)
        if diferencia < umbral:
            break
        contador += 1
        
    return contador +1
#####################################################################################################


####################################################################
####################################################################
############################BOTONES#################################
####################################################################
####################################################################

###############################Función que lee la frecuencia desde la ruleta#####################################
@app.callback(
    Output('slider-output-frequency', 'children'),
    Input('frequency-knob', 'value'))
def update_output(value):
    return '{} Hz'.format(np.round(value, 1))
#################################################################################################################

###############################Función que lee el número de golpes desde la ruleta###############################
@app.callback(
    Output('hitter-output-frequency', 'children'),
    Input('hitting-knob', 'value'))
def update_output(value):
    return '{} Hits'.format(int(np.round(value, 0)))
#################################################################################################################

#################################Función que lee la velocidad de los ventiladores#####################
@app.callback(
    Output('fan-velocity-output', 'children'),
    Input('fan-velocity', 'value'))
def update_output(value):
    return '{} Fan velocity'.format(int(np.round(value, 0)))
#################################################################################################################

#################################Función que envía la velocidad de los ventiladores al arduino####################
@app.callback(
    Output('send-fan-velocity-output', 'children'),
    Input('btn-nclicks-72', 'n_clicks'),
    Input('fan-velocity', 'value')
)
def send_fan_velocity(btn,value):  # n = number of horizontal line to move the robot
    if "btn-nclicks-72" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)
        print("********************************************")
        print("*********Send fan velocity **************")
        print("********************************************")
        MESSAGE='4 '+str(value)#Send velocity los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan velocity change: OK\n"
            print("*********Fan velocity change: OK*********")
        else:
            msg = "Fan velocity change: ERROR\n"
            print("*********Fan velocity change: ERROR*********")
        log = open('dynamic_text_files/logfile.txt', 'a')
        log.write(msg)
        return html.Div(msg)
#################################################################################################################


###############################Función que inicia el golpeo mediante el botón###################################
@app.callback(
    Output('start-solenoid-output', 'children'),
    Input('btn-nclicks-2', 'n_clicks'),
    Input('hitting-knob', 'value'),
    Input('frequency-knob', 'value')
)
def start_hitting(btn2, n, freq):  # n = number of hits, freq = frequency
    if "btn-nclicks-2" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)

        print("********************************************")
        print("*********Start hitting**********************")
        print("********************************************")
        number = np.round(n)
        MESSAGE='3 '+str(np.round(freq,1))+' '+str(number)#Activa el solenoide
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = " Finished hitting cycle: OK\n"
            print("********************************************")
            print("*********Finished hitting cycle: OK*********")
            print("********************************************")
        else:
            msg = " Finished hitting cycle: ERROR\n"
            print("********************************************")
            print("********Finished hitting cycle: ERROR*******")
            print("********************************************")
        log = open('dynamic_text_files/logfile.txt', 'a')
        log.write(str(datetime.datetime.now()))
        log.write(msg)
        log.write('Sending: PUNTO\n')
        log.close()
        return html.Div(msg)
##################################################################################################################
    
    
###############################Función que mueve el robot automáticamente#########################################
@app.callback(
    Output('start-automatic-output', 'children'),
    State('figura-store', 'data'),
    Input('btn-nclicks-3', 'n_clicks'),
    Input('hitting-knob', 'value'),
    Input('frequency-knob', 'value'),
    Input("slider-input-mesh-horizontal", "value"),
    Input("slider-input-mesh-vertical", "value"),
    Input("column-ini", "value"),
    Input("column-fin", "value"),
    Input("x-offset", "value"),
    Input("y-offset", "value"),
    Input("x-offset-gpr", "value"),
    Input("y-offset-gpr", "value")
)
def start_automatic(current_figure,btn2, n, freq, res_x, res_y, col_ini, col_fin, width, height, res_x_gpr, res_y_gpr):  # n = number of hits, freq = frequency
    if "btn-nclicks-3" == ctx.triggered_id:
        global s, stop_press, pause_press
        if s.is_open==False:
            s = serial.Serial(portname, 115200)

        #Grid selection
        msg = "Reading Grid selection"
        with open("Coordinates_Input/cc_start_grid.csv") as f:
            df = pd.read_csv(f)
        f.close()

        xcc = [float(i)/1000 for i in df['x'].tolist()]
        ycc = [float(i)/1000 for i in df['y'].tolist()]
        x=np.array(xcc)
        y=np.array(ycc)

        #######Calculamos el numero de puntos horizontales (res_x) y verticales (res_y) para que sea del fichero no de lo que se ha introducido para crear la malla
        umbral=0.300 #Valor en metros que se tendrá que actualizar
        res_y=contar_hasta_disminucion(y, umbral)
        res_x=int(len(x)/res_y)
        print("Numero de columnas (res_x): "+str(res_x))
        print("Número de filas (res_y): "+str(res_y))
        ###########################################################################
        
        print("**************************************BEGIN AUTOMATIC INSPECTION*****************************\n")
        print("FIRST COORDINATE\n")
        i=col_ini*res_y
        print("Number of inspection points: "+ str((col_fin+1)*res_y-i))

        if i > (res_x*res_y):
            msg="Initial column number exceeded\n"
            print(msg)
            return html.Div(msg)

        if ((col_fin+1)*res_y) > (res_x*res_y):
            msg="Final column number exceeded\n"
            print(msg)
            return html.Div(msg)

        #NUEVO
        #Desactivamos los ventiladores
        log = open('dynamic_text_files/logfile.txt', 'a')
        MESSAGE='1'#Desactiva los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan deactivated: OK\n"
            print("*********Fan deactivated: OK*********")
        else:
            msg = "Fan deactivated: ERROR\n"
            print("*********Fan deactivated: ERROR*********")
        log.write(msg)
        log.close()
            
                
        while i < (col_fin+1)*res_y:
            #Comprobamos si se ha dado al botón de pause
            if pause_press==1:

                #Desactivamos los ventiladores
                log = open('dynamic_text_files/logfile.txt', 'a')
                MESSAGE='1'#Desactiva los ventiladores
                msg_send=send_arduinoSolenoid(MESSAGE,host,port)
                if msg_send==0:
                    msg = "Fan deactivated: OK\n"
                    print("*********Fan deactivated: OK*********")
                else:
                    msg = "Fan deactivated: ERROR\n"
                    print("*********Fan deactivated: ERROR*********")
                log.write(msg)
                log.close()
                
                print("\n Last point executed: "+ str(i-1))
                res = input("""~~~~~~~Code interupted~~~~~~~
                \n Press R to resume
                \n Press Q to quit
                \n Press M to microsteps
                \n Press i the point number to restart the hitting""")
                if res == "R" :
                    print("R")
                    # pass and resume code
                    pass
                elif res == "Q" :
                    print("Q")
                    pause_press=0
                    #Save output and quit code
                    return
                elif res == "M":
                    res1="Yes"
                    while res1=="Yes":
                        x=input("x-adjustment (en m)")
                        y=input("y-adjustment (en m)")
                        
                        # Wake up grbl
                        log = open('dynamic_text_files/logfile.txt', 'a')
                        temp = "\r\n\r\n"
                        s.write(temp.encode())
                        time.sleep(0.01)  # Wait for grbl to initialize
                        s.flushInput()  # Flush startup text in serial input

                        #Desactivamos los ventiladores.
                        MESSAGE='1'#Desactiva los ventiladores
                        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
                        if msg_send==0:
                            msg = " Fan deactivated: OK\n"
                            print("*********Fan deactivated: OK*********")
                        else:
                            msg = " Fan deactivated: ERROR\n"
                            print("*********Fan deactivated: ERROR*********")
                        log.write(str(datetime.datetime.now()))
                        log.write(msg)

                        #Movemos el robot
                        if msg_send==0:
                            [X,Y]=mini_move(float(x), float(y))
                            #Remove last movement and put the new position
                            f_coord = open("Coordinates_Input/cc_start_grid.csv", "r")
                            data = f_coord.readlines()
                            data[i]= str(i-1) + ','+ str(X*1000)+','+ str(Y*1000) + '\n'
                            f_coord = open("Coordinates_Input/cc_start_grid.csv", "w")
                            f_coord.writelines(data)
                            f_coord.close()
                        else:
                            print("*********ERROR not microsteps done*********")
                        res1= input("Do you want to continue moving? (Yes/No)")
                    #Grid selection
                    msg = "Reading Grid selection"
                    with open("Coordinates_Input/cc_start_grid.csv") as f:
                        df = pd.read_csv(f)
                    f.close()
                    xcc = [float(i)/1000 for i in df['x'].tolist()]
                    ycc = [float(i)/1000 for i in df['y'].tolist()]
                    x=np.array(xcc)
                    y=np.array(ycc)
                    i=i-1
                else:
                    i=int(res)

            pause_press=0
            print("Point number: "+str(i))
            print("Column: "+str(int(i/res_y)))
            print('Point coordinates '+ str(i) +' : '+ str(x[i])+ ' , '+ str(y[i]))
            
            # Wake up grbl
            log = open('dynamic_text_files/logfile.txt', 'a')
            temp = "\r\n\r\n"
            s.write(temp.encode())
            time.sleep(0.01)  # Wait for grbl to initialize
            s.flushInput()  # Flush startup text in serial input
            

            time.sleep(3)#Tiempo de espera entre mover el robot hasta activar los ventiladores

            if msg_send==0:
                # Calculate new rope lengths for all motors
                new_rope_lengths = [calculate_rope_length(motor_pos, x[i], y[i]) for motor_pos in motor_positions]
                # Calculate the number of steps each motor needs to move
                motor_steps = [calculate_motor_steps(change, steps_per_revolution[i]) for i, change in enumerate(new_rope_lengths)]
                movetemp = "G90 X" + str(motor_steps[0]) + " Y" + str(motor_steps[1]) + movespeed + "\n"

                s.write(movetemp.encode())
                #Escribimos las coordenadas X y Y en el fichero log no las longitudes de las cuerdas
                command_x_y="G90 X" + str(x[i]) + " Y" + str(y[i])+ "\n"
                l = command_x_y.strip()  # Strip all EOL characters for consistency
                log.write(str(datetime.datetime.now()))
                log.write(' Sending: ' + l + '\n')
                print("New Rope Lengths:", new_rope_lengths)
                print("Motor Steps Required:", motor_steps)
                print("Commands for Arduino 1:", movetemp)

                #Calcula el tiempo de espera según la distancia que tenga que recorrer suponiendo que tarda 2 segundos en recorrer 0.5 m
                current_x = float(last_movement_searcher()[-1][-2])
                current_y = float(last_movement_searcher()[-1][-1])
                time_wait=round((math.sqrt((x[i]-current_x)**2+(y[i]-current_y)**2))*4) #0.5 m tarda 2 segundos
                print("Time wait till: "+ str(time_wait))
                time.sleep(time_wait)

                log.write('Sending: PUNTO \n')
                log.close()

                time.sleep(1)#Tiempo de espera entre mover el robot hasta activar los ventiladores


                number = np.round(n, 1)
                MESSAGE='0 '+str(np.round(freq))+' '+str(number)#Activamos los ventiladores y el solenoide
                msg_send=send_arduinoSolenoid(MESSAGE,host,port)
                if msg_send==0:
                    msg = " Finished hitting cycle: OK\n"
                    print("********************************************")
                    print("*********Finished hitting cycle: OK*********")
                    print("********************************************")
                else:
                    msg = " Finished hitting cycle: ERROR\n"
                    print("********************************************")
                    print("********Finished hitting cycle: ERROR*******")
                    print("********************************************")
                log = open('dynamic_text_files/logfile.txt', 'a')
                log.write(str(datetime.datetime.now()))
                log.write(msg)
                log.close()
                i=i+1
                print("i:")
                print(i)
                
        #Desactivamos los ventiladores cuando acaba el último punto
        log = open('dynamic_text_files/logfile.txt', 'a')
        MESSAGE='1'#Desactiva los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan deactivated: OK\n"
            print("*********Fan deactivated: OK*********")
        else:
            msg = "Fan deactivated: ERROR\n"
            print("*********Fan deactivated: ERROR*********")
        log.write(msg)
        log.close()
        print("**************************************FINAL AUTOMATIC INSPECTION*****************************\n")
        return html.Div('Finish')
###########################################################################################################

############################Función que define el origin de coordenadas (0,0)############################
@app.callback(
    Output('set-zero-output', 'children'),
    Input('btn-nclicks-200', 'n_clicks'),
    prevent_initial_call=False  # 👈 Esto permite que se ejecute al inicio
)
def set_zero(btn):
##    if "btn-nclicks-200" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)

        # Wake up grbl
        log = open('dynamic_text_files/logfile.txt', 'a')
        temp = "\r\n\r\n"
        s.write(temp.encode())
        time.sleep(0.01)  # Wait for grbl to initialize
        s.flushInput()  # Flush startup text in serial input
        x=0.0
        y=0.0
        # Calculate new rope lengths for all motors
        new_rope_lengths = [calculate_rope_length(motor_pos, x, y) for motor_pos in motor_positions]
        print("New Rope Lengths ORIGIN:", new_rope_lengths)
        # Calculate the number of steps each motor needs to move
        motor_steps = [calculate_motor_steps(change, steps_per_revolution[i]) for i, change in enumerate(new_rope_lengths)]
        print("Motor Steps Required ORIGIN:", motor_steps)
  
        command = "G92 X" + str(motor_steps[0]) + " Y" + str(motor_steps[1])+ "\n"

        s.write(command.encode())
        #Guardamos X e Y no las longitudes de las cuerdas que es en realidad lo que movemos los ejes X e Y
        command_x_y="G92 X" + str(x) + " Y" + str(y)+ "\n"
        l = command_x_y.strip()  # Strip all EOL characters for consistency
        log.write('Sending: ' + l + '\n')
        log.close()
        
        msg = "Current position saved as zero"
        print("*********ZERO position saved as zero*********")
        return html.Div(msg)
###########################################################################################################

##########################Botón Delete previous... Borra todos los datos del fichero logfile.txt############################
@app.callback(
    Output('delete-button-output', 'children'),
    Input('btn-nclicks-13', 'n_clicks')
)
def home(btn13):  # n = number of hits, freq = frequency
    if "btn-nclicks-13" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)
        msg = "All datapoints were deleted.\nReady for new test."
        log = open('dynamic_text_files/logfile.txt', 'a')
        log.truncate(0)
        log.close()
        return html.Div(msg)
############################################################################################################################
    
##########################Botón UNDO. Borra el último dato del fichero logfile.txt#################################################
@app.callback(
    Output('undo-button-output', 'children'),
    Input('btn-nclicks-123', 'n_clicks')
)
def home(btn123):  # n = number of hits, freq = frequency
    if "btn-nclicks-123" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)

        msg = "Last probe deleted."
        linenumber = df_maker_nuevo()[1][-1]
        print(linenumber)
        with open("dynamic_text_files/logfile.txt", "r") as f:
            lines = f.readlines()
            print('deleted line: ', lines[linenumber])
        with open("dynamic_text_files/logfile.txt", "w") as f:
            del lines[linenumber]
            for line in lines:
                f.write(str(line))

        return html.Div(msg)
############################################################################################################################
@app.callback(
    Output('figura-store', 'data'),
    Output('update_plot_fromfile', 'children'),
    State('figura-store', 'data'),
    Input('reload-btn', 'n_clicks')
)
def update_plot_fromfile(current_figure,n_clicks):
    if "reload-btn" == ctx.triggered_id:
        # Read X and Y only
        df = pd.read_csv("Coordinates_Input/cc_start_grid.csv")

        # Basic 2D scatter plot
        fig = go.Figure()

        fig.add_trace(go.Scatter(
            x=df['x'],
            y=df['y'],
            mode='markers',
            marker=dict(size=10, color='blue'),
            name='Mesh Points'
        ))

        x_cc = df['x'].tolist()
        y_cc = df['y'].tolist()
        x_cc = [x / 1000 for x in x_cc]
        y_cc = [x / 1000 for x in y_cc]

        #######Calculamos el numero de puntos horizontales (res_x) y verticales (res_y) para que sea del fichero no de lo que se ha introducido para crear la malla
         #Grid selection
        msg = "Reading Grid selection"
        with open("Coordinates_Input/cc_start_grid.csv") as f:
            df = pd.read_csv(f)
        f.close()
        x=np.array(x_cc)
        y=np.array(y_cc)
        umbral=0.300 #Valor en metros que se tendrá que actualizar
        res_y=contar_hasta_disminucion(y, umbral)
        res_x=int(len(x)/res_y)#Numero de puntos horizontales, numero columnas

        if res_x % 2==0:
            point_y=float(y_cc[-1-res_y])
        else:
            point_y=float(y_cc[-1])    
        ###########################################################################

        fig1 = go.Figure(go.Scatter(x=x_cc, y=y_cc, mode="markers"))
        fig2 = go.Figure(
            go.Scatter(x=[float(x_cc[0]), float(x_cc[-1]),float(x_cc[-1]), 0+float(x_cc[0]), float(x_cc[0])], y=[float(y_cc[0]), float(y_cc[0]), point_y, point_y, float(y_cc[0])],
                       fill="tonext"))
        #Pinta el punto que se está inspeccionando
        fig = go.FigureWidget(data=fig1.data + fig2.data)
        fig.update_xaxes(range=[-0.1, w_cnc])
        fig.update_traces(marker={'size': 15})
        fig.update_yaxes(range=[-0.1, h_cnc+0.1])
        fig.update_layout(template="plotly")
        fig.update_layout(showlegend=False)
        fig.update_yaxes(automargin='left+top+right+bottom')
        fig.update_layout(clickmode='event+select')
        fig.update_traces(marker_size=20)
        
        msg="Total number of columns: "+ str(res_x)

        return fig.to_dict(), html.Div(msg)
    return  no_update, no_update

############################################################################################################################
@app.callback(
    Output("plot", "figure"),
    Input('figura-store', 'data'),
    Input("slider-input-mesh-horizontal", "value"),
    Input("slider-input-mesh-vertical", "value"),
    Input("x-offset", "value"),
    Input("y-offset", "value"),
    Input("x-offset-gpr", "value"),
    Input("y-offset-gpr", "value")
)
def mesh_grid(fig_data, res_x, res_y, width, height, res_x_gpr, res_y_gpr):
    global x_cc,y_cc

    #Comprobamos si alguna variable es NONE y si alguna lo es devuelve un grid vacío. Esto quita el error al meter los datos
    if any(v is None for v in [res_x, res_y, width,height, res_x_gpr, res_y_gpr ]):
        print("Valor NONE"+str([res_x, res_y, width,height, res_x_gpr, res_y_gpr ]))
        return go.Figure()
    try:
        w = float(width)-2*float(res_x_gpr)
        h = float(height)-float(res_y_gpr)
        dx = w / (res_x-1)
        dy = h / (res_y-1)
    except:
        return go.Figure()    

        
    x_lst = [0+float(res_x_gpr)]
    for i in range(res_x-2):
        x_cc = round(dx * (i + 1),4)
        x_lst.append(x_cc+float(res_x_gpr))
    x_lst.append(w+float(res_x_gpr))
    y_lst = [0]
    for i in range(res_y-2):
        y_cc = round(dy * (i + 1),4)
        y_lst.append(y_cc)
    y_lst.append(h)

    x_cc = []
    y_cc = []
    for x in x_lst:
        for y in y_lst:
            x_cc.append(x)
            y_cc.append(y)

    fig1 = go.Figure(go.Scatter(x=x_cc, y=y_cc, mode="markers"))
    fig2 = go.Figure(
        go.Scatter(x=[0+float(res_x_gpr), w+float(res_x_gpr), w+float(res_x_gpr), 0+float(res_x_gpr), 0+float(res_x_gpr)], y=[0, 0, h, h, 0],
                   fill="tonext"))
    #Pinta el punto que se está inspeccionando
    if fig_data:
        fig = go.Figure(fig_data)
    else:
        fig = go.FigureWidget(data=fig1.data + fig2.data)

    fig.update_xaxes(range=[-0.1, w_cnc])
    fig.update_traces(marker={'size': 15})
    fig.update_yaxes(range=[-0.1, h_cnc+0.1])
    fig.update_layout(template="plotly")
    fig.update_layout(showlegend=False)
    fig.update_yaxes(automargin='left+top+right+bottom')
    fig.update_layout(clickmode='event+select')
    fig.update_traces(marker_size=20)
    
    # Automatically rescale axes
    fig.update_layout(
        xaxis=dict(autorange=True),
        yaxis=dict(autorange=True),
##        margin=dict(l=0, r=0, t=0, b=0)
    )
    
    return fig


###########################Función para mover punto a punto manualmente el robot#####################################
@app.callback(
    Output('click-data', 'children'),
    Input('plot', 'clickData'))
def display_click_data(clickData):
    if str(json.dumps(clickData, indent=2)) == "null":
        mssg = "No point has been selected.\nPlease click on a blue dot in order to move"
    else:
        print("********************************************")
        print("*********Start moving point**********************")
        print("********************************************")
        # extract location data from
        txt = json.dumps(clickData, indent=2)
        xtxt = txt.splitlines()[6]
        ytxt = txt.splitlines()[7]
        x = float(xtxt[11:-1]) #* 1000
        y = float(ytxt[11:-1]) #* 1000
        print('\n*********Selected/clicked position [x] [y]:*********')
        print(x, y)
        # Wake up grbl
        log = open('dynamic_text_files/logfile.txt', 'a')
        temp = "\r\n\r\n"
        s.write(temp.encode())
        time.sleep(0.01)  # Wait for grbl to initialize
        s.flushInput()  # Flush startup text in serial input
        #Cambio de Velocidad X (110) e Y(111), cambio de aceleración X(120) e y(121)
        temp = "$110=750\n$111=750\n$120=10\n$121=10\n"
        s.write(temp.encode())
        time.sleep(0.01)  # Wait for grbl to initialize
        s.flushInput()  # Flush startup text in serial input
        #Desactivamos los ventiladores
        MESSAGE='1'#Desactiva los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan deactivated: OK\n"
            print("*********Fan deactivated: OK*********")
            log = open('dynamic_text_files/logfile.txt', 'a')
            log.write(msg)
            # Calculate new rope lengths for all motors
            new_rope_lengths = [calculate_rope_length(motor_pos, x, y) for motor_pos in motor_positions]
            print("New Rope Lengths:", new_rope_lengths)
            # Calculate the number of steps each motor needs to move
            motor_steps = [calculate_motor_steps(change, steps_per_revolution[i]) for i, change in enumerate(new_rope_lengths)]
            print("Motor Steps Required:", motor_steps)
            command = "G90 X" + str(round(motor_steps[0],3)) + " Y" + str(round(motor_steps[1],3)) + movespeed + "\n"
            print(command)
            s.write(command.encode())
            command_x_y="G90 X" + str(x) + " Y" + str(y)+ "\n"
            l = command_x_y.strip()  # Strip all EOL characters for consistency
            log.write('Sending: ' + l + '\n')
            #Para poder cambiar el fichero start_grid con microsteps
            dato1 = str(x*1000)
            dato2 = str(y*1000)
            print("Dato1: "+ str(dato1))
            print("Dato2: "+ str(dato2))
            with open("Coordinates_Input/cc_start_grid.csv", newline='') as f:
                reader = csv.reader(f)
                for fila in reader:
                    if dato1 == fila[1] and dato2== fila[2]:
                        print("Fila encontrada:", fila[0])
                        log.write('DISPLAY: ' + fila[0]+ '\n')
            log.close()
            mssg = "*********Arrived at: x = ", x, "   y = ", y, "   [m]*********"
        else:
            msg = "Fan deactivated: ERROR\n"
            print("*********Fan deactivated: ERROR*********")
            mssg = "*********Message not arrived at: x = ", x, "   y = ", y, "   [m]*********"
        print("**********************************************************************")
        print(mssg)
        print("**********************************************************************")
    return mssg
###########################################################################################################

###########################Función para activar los ventiladores manualmente#####################################
@app.callback(
    Output('position-solenoid-button', 'children'),
    Input('btn-nclicks-1', 'n_clicks')
)
def hit(btn1):
    ##ACTIVAMOS LOS VENTILADORES (Enviamos un 2 al arduino)
    if "btn-nclicks-1" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)
        print("********************************************")
        print("*********Start activating fans**************")
        print("********************************************")
        MESSAGE='2'#Activa los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan activated: OK\n"
            print("*********Fan activated: OK*********")
        else:
            msg = "Fan activated: ERROR\n"
            print("*********Fan activated: ERROR*********")
        log = open('dynamic_text_files/logfile.txt', 'a')
        log.write(msg)
        return html.Div(msg)
#####################################################################################################

###########################Función para hacer un reset de los ventiladores manualmente#####################################
@app.callback(
    Output('reset-fan-button', 'children'),
    Input('btn-nclicks-71', 'n_clicks')
)
def hit(btn1):
    ##HACEMOS UN RESET (Enviamos un 5 al arduino)
    if "btn-nclicks-71" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)
        print("********************************************")
        print("*********Reset fans**************")
        print("********************************************")
        MESSAGE='5'#Reset los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan reset: OK\n"
            print("*********Fan activated: OK*********")
        else:
            msg = "Fan reset: ERROR\n"
            print("*********Fan reset: ERROR*********")
        log = open('dynamic_text_files/logfile.txt', 'a')
        log.write(msg)
        return html.Div(msg)
#####################################################################################################



###########################Generate grid to start the automatic hitting##############################
@app.callback(
    Output('start-grid-output', 'children'),
    Input("slider-input-mesh-horizontal", "value"),
    Input("slider-input-mesh-vertical", "value"),
    Input('btn-nclicks-9', 'n_clicks')
)
def hit(res_x, res_y, btn1):
    if "btn-nclicks-9" == ctx.triggered_id:
        msg = "Generate grid file to run program automatically, total number of columns: "+ str(res_x)
        global s
        grid_lst=[]
        x_aux = []
        y_aux = []
        y_aux2 = []
        #Para que me genere el fichero de movimiento como una serpiente. Primera fila empezando en 0,
        #segunda fila empezando en el punto más lejano del origen x, tercera fila empezando en el x0, etc..
        #Ordenamos los datos según las coordenadas x siguiendo el esquema descrito, según la fila sea par o impar

        
        #Cambiamos el grid para que sea recorrido en forma de serpiente pero por columnas no por filas
        x_aux.extend([x_cc[i] * 1000 for i in range(0,(res_x*res_y))])

        for j in range(0,res_x):
            if (j % 2) == 0:
                y_aux.extend([y_cc[i] * 1000 for i in range(j*res_y,j*res_y+res_y)])  

            else:
                y_aux.extend([y_cc[i] * 1000 for i in range(j*res_y+res_y-1,j*res_y-1,-1)])

        x_grid_cc= x_aux
        y_grid_cc= y_aux
        grid_lst={'x': x_grid_cc, 'y': y_grid_cc}
        df_grid = pd.DataFrame(grid_lst, columns=['x', 'y'])
        
        df_grid.to_csv('Coordinates_Input/cc_start_grid.csv', sep=',')
        return html.Div(msg)
#####################################################################################################
    
###########################Función que desactiva los ventiladores#####################################
@app.callback(
    Output('solenoid-backward-button', 'children'),
    Input('btn-nclicks-7', 'n_clicks')
)
def hit(btn1):
    #DESACTIVAMOS LOS VENTILADORES
    if "btn-nclicks-7" == ctx.triggered_id:
        global s
        if s.is_open==False:
            s = serial.Serial(portname, 115200)
        print("********************************************")
        print("*********Start deactivating fans************")
        print("********************************************")
        MESSAGE='1'#Desactiva los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan deactivated: OK\n"
            print("*********Fan deactivated: OK*********")
        else:
            msg = "Fan deactivated: ERROR\n"
            print("*********Fan deactivated: ERROR*********")
        log = open('dynamic_text_files/logfile.txt', 'a')
        log.write(msg)
        log.close()
        return html.Div(msg)
#####################################################################################################

###########################Función para parar el programa############################################
@app.callback(
    Output('stop-button', 'children'),
    Input('btn-nclicks-5', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-5" == ctx.triggered_id:
        msg = "Program stopped"
        global s, stop_press, pause_press
        stop_press=1
        pause_press=1
        #Primero desactivamos el ventilador
        MESSAGE='1'#Desactiva los ventiladores
        msg_send=send_arduinoSolenoid(MESSAGE,host,port)
        if msg_send==0:
            msg = "Fan deactivated: OK\n"
            print("*********Fan deactivated: OK*********")
        else:
            msg = "Fan deactivated: ERROR\n"
            print("*********Fan deactivated: ERROR*********")
        print("*********Program stopped*********")
        s.close()
        return html.Div(msg)
#####################################################################################################
    
###########################Función para pausar el programa#############################################
@app.callback(
    Output('pause-button', 'children'),
    Input('btn-nclicks-6', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-6" == ctx.triggered_id:
        msg = "Program paused"
        global pause_press
        pause_press=1
        print("*********Program paused*********")
        return html.Div(msg)

################Funciones para los micropasos. Mover manualmente el robot cmt_to_move en cada eje##########
# X microsteps
@app.callback(
    Output('x-micro', 'children'),
    Input('btn-nclicks-20', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-20" == ctx.triggered_id:
        mini_move(x=cm_to_move, y=0)# cada click movemos 0.5 cm
        print("*********2 cm X movement done*********")
        return
    
# -X microsteps
@app.callback(
    Output('-x-micro', 'children'),
    Input('btn-nclicks-21', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-21" == ctx.triggered_id:
        mini_move(x=-cm_to_move, y=0)# cada click movemos -0.5 cm
        print("*********2 cm -X movement done*********")
        return
    
# Y microsteps   
@app.callback(
    Output('y-micro', 'children'),
    Input('btn-nclicks-22', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-22" == ctx.triggered_id:
        mini_move(x=0, y=cm_to_move)# cada click movemos 0.5 cm
        print("*********2 cm Y movement done*********")
        return
    
# -Y microsteps
@app.callback(
    Output('-y-micro', 'children'),
    Input('btn-nclicks-23', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-23" == ctx.triggered_id:
        mini_move(x=0, y=-cm_to_move)# cada click movemos 0.5 cm
        print("*********2 cm -Y movement done*********")
        return

# Save coordinates
@app.callback(
    Output('save-coordinates', 'children'),
    Input('btn-nclicks-40', 'n_clicks')
)
def hit(btn1):
    if "btn-nclicks-40" == ctx.triggered_id:
        last_mini_move_movement_searcher()
        msg="Coordinates have been saved"
        return html.Div(msg)
#####################################################################################################    


################################Dowload dataframe as excel file######################################  
@app.callback(
    Output("download-dataframe-xlsx", "data"),
    Input("btn-nclicks-12", "n_clicks"),
    prevent_initial_call=True,
)
def func(n_clicks):
##    df = df_maker()[0]
    df = df_maker_nuevo()[0]#CAMBIADO SOFIA
    return dcc.send_data_frame(df.to_csv, "cc.csv")
#####################################################################################################   

if __name__ == '__main__':
    app.run(debug=False)



