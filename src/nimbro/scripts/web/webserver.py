#!/usr/bin/python
# Author: Hafez Farazi <farazi@ais.uni-bonn.de>
import os
import commands
import threading
import copy
import web
import shutil
from web import form
import time
import re
import base64
from threading import Thread
import subprocess
import pexpect
import roslib
import rospy
import re
from gait_msgs.msg import GaitCommand
from head_control.msg import LookAtTarget
from nimbro_op_interface.msg import Buzzer
from robotcontrol.msg import RobotHeading
#from vision_module.msg import vision_outputs
from robotcontrol.msg import Diagnostics
from config_server.msg import ParameterValueList
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
#from vision_module.msg import localization_input
import datetime
from ansi2html import Ansi2HTMLConverter  # sudo pip install git+https://github.com/ralphbean/ansi2html
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
from flask import Flask, render_template, Response  # sudo pip install Flask
import math
from __builtin__ import True, False
import yaml
import rospkg
import signal
import sys
import numpy as np
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
        
class ThreadedCommand(object):
	def __init__(self, command, wait0, wait1, daemon=True, buffersize=100, interval=0.001):
	        self.interval = interval
	        self.command = command
	        self.buffersize = buffersize
	        self.p = None
	        thread = threading.Thread(target=self.run, args=())
	        thread.daemon = daemon  # Daemonize thread
	        thread.start()  # Start the execution
	        self.outputG = ""
	        self.outputTotalG = ""
	        self.killed = False
	        self.outputGCounter = 0
	        self.outputTotalGCounter = 0
	        self.outputGHtml = ""
	        self.outputTotalGHtml = ""
	        self.wait0 = wait0
	        self.wait1 = wait1
	def run(self):
		try:
			self.p = pexpect.spawn(self.command, timeout=None, maxread=999999)
			c = 0
			while self.p.isalive():  # not p.eof():
				strLine = self.p.readline()
				if strLine.strip():
					self.outputGCounter = 0
					self.outputTotalGCounter = 0
					c += 1
					self.outputG += strLine
					self.outputTotalG += strLine
					if(self.buffersize > 0 and c > self.buffersize):
						c = 0
						lines = self.outputG.split("\n")
						length = len(lines)
						if(length > self.buffersize):
							self.outputG = '\n'.join(lines[length - self.buffersize:])
				time.sleep(self.interval)
			self.outputG += "It is not alive!"
		except Exception, err:
			print ('ERROR: %sn' % str(err))
		except:
			print "ERRRRRROOOROROROROR"
	def kill(self):
		self.killed = True
		print "ThreadedCommand:kill" + str(self.p.pid)
		try:
			self.p.close(False);
 			time.sleep(self.wait0)
# 			self.p.close(True); # I commented this to prevent closing launch file which might be in the middle of killing other nodes
# 			time.sleep(self.wait1)
		except:
			pass
	def __exit__(self):
		print "ThreadedCommand:__exit__"
		self.kill()
	def __del__(self):
		print "ThreadedCommand:__del__"
		self.kill()
	def __eq__(self, other):
		print "ThreadedCommand:__eq__"
		self.kill()
		
jpeg = None
rCounter = 0
def callbackimg(data):
	global jpeg
	global rCounter
	try:
		rCounter += 1
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		if cv_image is not None:
			ret, jpeg2 = cv2.imencode('.jpg', cv_image)
			jpeg = jpeg2.tostring()
	except CvBridgeError as e:
		pass


headingData = 0
headingDataRadian = 0
def callbackheading(data):
	global headingData
	global headingDataRadian
	try:
		headingDataRadian = data.heading
		headingData = math.degrees(data.heading)
	except :
		pass
	
locData = [0, 0, 0]
def callbackLoc(data):
	global locData
	try:
		locData[0] = data.location.position.x
		locData[1] = data.location.position.y
		locData[2] = data.location.position.z
	except :
		pass
	
lastDiagnostics = None
diagnosticsData = None
def callbackdiagnostics(data):
	global lastDiagnostics
	global diagnosticsData
	try:
		lastDiagnostics = time.time()
		diagnosticsData = data
	except :
		pass
	
lastParam = None
paramData = None
def callbackparam(data):
	global lastParam
	global paramData
	try:
		lastParam = time.time()
		paramData = data
	except :
		pass
	
def timer_callback(event):
	global lastAccessTime
	global image_sub_active
	global image_sub
	if((datetime.datetime.now()-lastAccessTime).total_seconds()>30):
		if(image_sub_active):
			print "Deactive image_sub"
			image_sub.unregister()
			image_sub_active=False
	elif (not image_sub_active):
		print "Active image_sub"
		image_sub = rospy.Subscriber("/vision/webImg", Image, callbackimg)
		image_sub_active=True
def getjpeg():
	global jpeg
	return jpeg

appFlask = Flask(__name__)
def gen():
	while True:
		yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + getjpeg() + b'\r\n\r\n')
        
@appFlask.route('/video_vision')
def video_feed():
	return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

def runFlask():
	appFlask.run(host='0.0.0.0', debug=False)

CQC = []
CQC.append(["roslaunch launch webserver.launch", "Robotcontrol", 300, 1, 1])
CQC.append(["roslaunch launch vision_module.launch", "Vision", 60, 0, 0])
CQC.append(["roslaunch launch behaviour.launch", "Behaviour", 60, 0, 0])
CQC.append(["roslaunch launch robot_tracker.launch", "Tracker", 60, 0, 0])
CQC.append(["killall", "Killall", 10, 0, 0])
CQC.append(["resetall", "Resetall", 10, 0, 0])
CQC.append(["top -b", "Top", 700, 0, 0])
CQC.append(["reboot", "Reboot", 10, 0, 0])
CQC.append(["poweroff", "Poweroff", 10, 0, 0])
MAXNUMBER = len(CQC)
CQ = [None] * MAXNUMBER
rospack = rospkg.RosPack()
time.sleep(1)
rospy.init_node('webserver_node', anonymous=True)

mf = file(rospack.get_path('launch') + "/config/field_model.yaml", 'r')
wifiConfigAddress = rospack.get_path('launch')+"/network/communication.launch"
wifiAddress = rospack.get_path('scripts').replace("/share/","/lib/")+"/wifi.py"
ymlData = yaml.safe_load(mf)
if('field_type' not in ymlData):
	print "Something is wrong with file ->" + mf
	sys.exit()
type = ymlData['field_type']
if('bonn' != type and 'teensize' != type and 'kidsize' != type):
	type = 'teensize'

if('length' not in ymlData[type] or 'width' not in ymlData[type] or 'goalAreaDepth' not in ymlData[type] or 'goalAreaWidth' not in ymlData[type] or 'penaltyMarkerDist' not in ymlData[type] or 'centerCircleDiameter' not in ymlData[type] or 'goalWidth' not in ymlData[type] or 'boundary' not in ymlData[type]):
	print "Something is wrong with file ->" + mf
	sys.exit()
A = ymlData[type]['length']
B = ymlData[type]['width']
E = ymlData[type]['goalAreaDepth']
F = ymlData[type]['goalAreaWidth']
G = ymlData[type]['penaltyMarkerDist']
H = ymlData[type]['centerCircleDiameter']
D = ymlData[type]['goalWidth']
I = ymlData[type]['boundary']
scale = 5;
marginW = int(I * 10 * scale)
marginH = int(I * 10 * scale)
imgE = int(E * 10 * scale)
imgF = int(F * 10 * scale)
imgD = int(D * 10 * scale)
Width = int(A * 10 * scale)
Height = int(B * 10 * scale)
Width2 = int(A * 5 * scale)
Height2 = int(B * 5 * scale)
Radius = int(H * 5 * scale)
locW = int(Width + (marginW * 2))
locH = int(Height + (marginH * 2))
P1 = str(marginW) + ',' + str(marginH)
P2 = str(marginW + Width) + ',' + str(marginH)
P3 = str(marginW + Width) + ',' + str(Height + marginH)
P4 = str(marginW) + ',' + str(Height + marginH)
P5 = str(marginW + Width2) + ',' + str(marginH)
P6 = str(marginW + Width2) + ',' + str(Height + marginH)
P7 = str(marginW + imgE) + ',' + str(marginH + int((Height - imgF) / 2.))
P8 = str(marginW + imgE) + ',' + str(marginH + int((Height - imgF) / 2.) + imgF)
P9 = str(marginW) + ',' + str(marginH + int((Height - imgF) / 2.))
P10 = str(marginW) + ',' + str(marginH + int((Height - imgF) / 2.) + imgF)
PO7 = str(marginW + Width - imgE) + ',' + str(marginH + int((Height - imgF) / 2.))
PO8 = str(marginW + Width - imgE) + ',' + str(marginH + int((Height - imgF) / 2.) + imgF)
PO9 = str(marginW + Width) + ',' + str(marginH + int((Height - imgF) / 2.))
PO10 = str(marginW + Width) + ',' + str(marginH + int((Height - imgF) / 2.) + imgF)
P11 = str(marginW) + ',' + str(marginH + int((Height - imgD) / 2.))
P12 = str(marginW) + ',' + str(marginH + int((Height - imgD) / 2.) + imgD)
PO11 = str(marginW + Width) + ',' + str(marginH + int((Height - imgD) / 2.))
PO12 = str(marginW + Width) + ',' + str(marginH + int((Height - imgD) / 2.) + imgD)

head_pub = rospy.Publisher("/headcontrol/target", LookAtTarget, queue_size=1)
gait_pub = rospy.Publisher("/gaitCommand", GaitCommand, queue_size=1)
gait_beh_pub = rospy.Publisher("/walk_and_kick/stoppedGaitCmd", GaitCommand, queue_size=1)
#pos_pub = rospy.Publisher("/vision/setLocation", localization_input, queue_size=1)
buz_pub = rospy.Publisher("/nimbro_op_interface/buzzer", Buzzer, queue_size=1)
# time.sleep(1)
bridge = CvBridge()
image_sub_active=True
image_sub = rospy.Subscriber("/vision/webImg", Image, callbackimg)
heading_sub = rospy.Subscriber("/robotmodel/robot_heading", RobotHeading, callbackheading)
#loc_sub = rospy.Subscriber("/vision/outputs", vision_outputs, callbackLoc)
diagnostics_sub = rospy.Subscriber("/robotcontrol/diagnostics", Diagnostics, callbackdiagnostics)
params_sub = rospy.Subscriber("/config_server/parameter_values", ParameterValueList, callbackparam)
timer_obj = rospy.Timer(rospy.Duration(1), timer_callback)

full_path = os.path.realpath(__file__)
basePath = full_path.replace(__file__, "")

t_globals = dict(
	datestr=web.datestr,
)
render = web.template.render('webFiles' + '/', cache=False, globals={'zip':zip, 'time':time, 'enumerate':enumerate, 'str':str})
    
urls = (
	'/(.*)', 'index'
)
lastAccessTime=datetime.datetime.now()
formp = form.Form()
allowed = (
    ('test', 'test')
)
gpan = 0
gtilt = 0
	
def GetUsername():
	auth = web.ctx.env.get('HTTP_AUTHORIZATION')
	username = "None"
	if  auth is not None:
		auth = re.sub('^Basic ', '', auth)
		username, password = base64.decodestring(auth).split(':')
		if (username) in allowed:
			return username;
	
	return username
def GetUserPass():
	auth = web.ctx.env.get('HTTP_AUTHORIZATION')
	username = "None"
	if  auth is not None:
		auth = re.sub('^Basic ', '', auth)
		username, password = base64.decodestring(auth).split(':')
		if (username) in allowed:
			return username;
	
	return username,password
class Stack:
	def __init__(self):
		self.__storage = []
	
	def isEmpty(self):
		return len(self.__storage) == 0
	
	def push(self, p):
		self.__storage.append(p)
	
	def pop(self):
		return self.__storage.pop()
	
	def __len__(self):
		return len(self.__storage)
	
	def __getitem__(self, i):
		return self.__storage[i]
   
formp = form.Form()

def controlIsAlive():
	global lastDiagnostics
	global diagnosticsData
	if lastDiagnostics is not None and time.time() - lastDiagnostics < 7 and diagnosticsData is not None :
		return True
	else:
		return False
	

def configBar():
	
	downloadConf = ""
	try:
		robotName = subprocess.check_output("rosparam get /config_server/robot_name", shell=True).strip()
		downloadConf = "<a download='config_" + robotName + ".yaml' href='/config_" + robotName + ".yaml'>Download config_" + robotName + ".yaml</a>"
	except:
		pass
	javacript = '<script type="text/javascript">'
	javacript += '''
		function retreivePass()
		{
			var user=document.getElementById("usernameIn").value;
			var pass=document.getElementById("passwordIn").value;
			
			   var xmlhttp;
			  if (window.XMLHttpRequest) {
			    xmlhttp = new XMLHttpRequest();
			  } else {
			    // code for older browsers
			    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
			  }
			  xmlhttp.onreadystatechange = function() {
			    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
			      document.getElementById("demo").innerHTML =
			      xmlhttp.responseText;
			    }
			  };
			  xmlhttp.open("GET", "''' + web.ctx.home + '''/retreiveconfig?username="+user+"&password="+pass, false);
			  xmlhttp.send();
			  alert(xmlhttp.responseText,false);
		}
		function deployPass()
		{
			document.getElementById("deployBtn").value="Please Wait ...";
			document.getElementById("deployBtn").enable=false;
			var user=document.getElementById("usernameIn").value;
			var pass=document.getElementById("passwordIn").value;
			
			   var xmlhttp;
			  if (window.XMLHttpRequest) {
			    xmlhttp = new XMLHttpRequest();
			  } else {
			    // code for older browsers
			    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
			  }
			  xmlhttp.onreadystatechange = function() {
			    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
			      document.getElementById("demo").innerHTML =
			      xmlhttp.responseText;
			    }
			  };
			  xmlhttp.open("GET", "''' + web.ctx.home + '''/deploy?username="+user+"&password="+pass, false);
			  xmlhttp.send();
			  alert(xmlhttp.responseText,false);
		}
		function loginGen()
		{
			var OSName="Unknown OS";
			if (navigator.platform.indexOf("Win")!=-1) OSName="Windows";
			if (navigator.platform.indexOf("Mac")!=-1) OSName="MacOS";
			if (navigator.platform.indexOf("X11")!=-1) OSName="UNIX";
			if (navigator.platform.indexOf("Linux")!=-1) OSName="Linux";
			
			if(OSName=="Linux")
			{
//				document.getElementById("login").innerHTML='Username: <input type="text" style="width:75px;" id="usernameIn">&nbsp;&nbsp;Password: <input type="password" style="width:75px;" id="passwordIn">&nbsp;<input type="submit" id="retieveBtn" value="Retreive" onclick="retreivePass()" />&nbsp;<input id="deployBtn" type="submit" value="Deploy" onclick="deployPass()" />&nbsp;&nbsp;'
				document.getElementById("login").innerHTML='<input type="hidden" style="width:75px;" id="usernameIn"> <input type="hidden" style="width:75px;" id="passwordIn">&nbsp;<input type="submit" id="retieveBtn" value="Retreive" onclick="retreivePass()" />&nbsp;<input id="deployBtn" type="submit" value="Deploy" onclick="deployPass()" />&nbsp;&nbsp;'
			}
		}loginGen();'''

	javacript += "</script>"
	return '''<style type="text/css">
		#modalContainer {
			background-color:transparent;
			position:absolute;
			width:100%;
			height:100%;
			top:0px;
			left:0px;
			z-index:10000;
			/*#background-image:url(tp.png);  required by MSIE to prevent actions on lower z-index elements */	
		}
		#alertBox {
			position:relative;
			width:400px;
			min-height:400px;
			margin-top:50px;
			border:2px solid #000;
			background-color:#F2F5F6;
			background-image:url(/imgs/alert.png);
			background-repeat:no-repeat;
			background-position:20px 30px;
		}
		#modalContainer > #alertBox {
			position:fixed;
		}
		#alertBox h1 {
			margin:0;
			font:bold 0.9em verdana,arial;
			background-color:#666699;
			color:#FFF;
			border-bottom:1px solid #000;
			padding:2px 0 2px 5px;
		}
		#alertBox p {
			font:0.7em verdana,arial;
			height:50px;
			padding-left:5px;
			margin-left:55px;
		}
		#alertBox #closeBtn {
			display:block;
			position:relative;
			margin:10px auto;
			padding:3px;
			border:2px solid #000;
			width:70px;
			font:0.7em verdana,arial;
			text-transform:uppercase;
			text-align:center;
			color:#FFF;
			background-color:#003300;
			text-decoration:none;
		}
		#cancelBtn {
			display:block;
			position:relative;
			margin:3px auto;
			padding:3px;
			border:2px solid #000;
			width:70px;
			font:0.7em verdana,arial;
			text-transform:uppercase;
			text-align:center;
			color:#FFF;
			background-color:#4d001f;
			text-decoration:none;
		}</style>''' + '''<script type="text/javascript">var ALERT_TITLE = "Attention";
		if(document.getElementById) {
			window.alert = function(txt,redirect) {
				createCustomAlert(txt,redirect);
			}
		}
		function createCustomAlert(txt,redirect) {
			d = document;
			if(d.getElementById("modalContainer")) return;
			mObj = d.getElementsByTagName("body")[0].appendChild(d.createElement("div"));
			mObj.id = "modalContainer";
			mObj.style.height = d.documentElement.scrollHeight + "px";
			alertObj = mObj.appendChild(d.createElement("div"));
			alertObj.id = "alertBox";
			if(d.all && !window.opera) alertObj.style.top = document.documentElement.scrollTop + "px";
			alertObj.style.left = (d.documentElement.scrollWidth - alertObj.offsetWidth)/2 + "px";
			alertObj.style.visiblity="visible";
			h1 = alertObj.appendChild(d.createElement("h1"));
			h1.appendChild(d.createTextNode(ALERT_TITLE));
			msg = alertObj.appendChild(d.createElement("p"));
			//msg.appendChild(d.createTextNode(txt));
			msg.innerHTML = txt;
			btn = alertObj.appendChild(d.createElement("a"));
			btn.id = "closeBtn";
			btn.appendChild(d.createTextNode("OK"));
			btn.href = "#";
			btn.focus();
			btn.onclick = function() { removeCustomAlert();
			if(redirect)
			{
				location.reload();
			}
			document.getElementById("deployBtn").value="Deploy";
			document.getElementById("deployBtn").enable=true;
			return false; }
			alertObj.style.display = "block";
		}
		function removeCustomAlert() {
			document.getElementsByTagName("body")[0].removeChild(document.getElementById("modalContainer"));
		}
		function loadXMLDoc(txt) {
		  var xmlhttp;
		  if (window.XMLHttpRequest) {
		    xmlhttp = new XMLHttpRequest();
		  } else {
		    // code for older browsers
		    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
		  }
		  xmlhttp.onreadystatechange = function() {
		    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
		      document.getElementById("demo").innerHTML =
		      xmlhttp.responseText;
		    }
		  };
		  xmlhttp.open("GET", "''' + web.ctx.home + '''/callservice?id="+txt, false);
		  xmlhttp.send();
		  alert(xmlhttp.responseText,txt=='9');
		}</script>''' + '''&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span id=login></span>''' + javacript+'''
		&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span id=login></span>&nbsp;&nbsp;
		|&nbsp;&nbsp;<button style="background-color:lightblue" onclick="loadXMLDoc('18')">Connect Wifi</button>&nbsp;&nbsp;<button style="background-color:lightblue" onclick="loadXMLDoc('19')">Block Wifi</button>&nbsp;&nbsp;<button style="background-color:lightblue" onclick="loadXMLDoc('20')">Reconnect Wifi</button>&nbsp;&nbsp;|&nbsp;&nbsp;''' + ("(No Robotcontrol is running)" if not CQ[0] else '''<button onclick="loadXMLDoc('8')">Save Config</button>&nbsp;<button onclick="loadXMLDoc('9')">Reset Config</button> &nbsp;&nbsp;|&nbsp;&nbsp;<button onclick="loadXMLDoc('16')">Reload Motions</button>&nbsp;&nbsp;|&nbsp;&nbsp;<button onclick="loadXMLDoc('17')">Set GC IP</button>''') + "&nbsp;&nbsp;|&nbsp;&nbsp;" + downloadConf + "<br><hr>"
    	 
def configBarPage(myBody):
	return '''<!doctype html>
	<html> <head> </head>
	<body > ''' + menuBarWithConfig() + myBody + '</body>'
    	 

def menuBar():
	global CQC
	global lastDiagnostics
	global diagnosticsData
	result = ""
	try:
		ifcRes=subprocess.check_output("ifconfig",shell=True)
		regEx=re.search(r'wlan\d',ifcRes.rstrip())
		if regEx:
		  	result = subprocess.check_output("iwgetid "+regEx.group(), shell=True)
    	except:
    		pass
	htmlSTR = '<h3><img  src="/imgs/logo.gif" />&nbsp;'

	if(lastDiagnostics is not None and time.time() - lastDiagnostics < 7 and diagnosticsData is not None):
		battryData = diagnosticsData.batteryVoltage
		commentsmsgs = str(round(battryData, 1))
		RobotType=os.environ['NIMBRO_ROBOT_TYPE']
		BatteryPercentArray=[16.4,15.8,15.4,15.1,14.9] # It is for 4 cell battery, For info See: http://www.instructables.com/id/Lithium-Polymer-Etiquette/
		if("D1" == RobotType):
			 BatteryPercentArray=map(lambda x:x+3.7, BatteryPercentArray)# D1 has 5 cell battery 
		if(battryData > BatteryPercentArray[0]):
			htmlSTR += '<a title="' + commentsmsgs + '"> <img  src="/imgs/p_100.jpg" alt="Battry Full" width=30 height=60" /></a>&nbsp;'		
		elif(battryData > BatteryPercentArray[1]):
			htmlSTR += '<a title="' + commentsmsgs + '"> <img  src="/imgs/p_80.jpg" alt="Battry OK" width=30 height=60" /></a>&nbsp;'
		elif(battryData > BatteryPercentArray[2]):
			htmlSTR += '<a title="' + commentsmsgs + '"> <img  src="/imgs/p_60.jpg" alt="Battry OK" width=30 height=60" /></a>&nbsp;'
		elif(battryData > BatteryPercentArray[3]):
			htmlSTR += '<a title="' + commentsmsgs + '"> <img  src="/imgs/p_40.jpg" alt="Battry Low" width=30 height=60" /></a>&nbsp;'
		elif(battryData > BatteryPercentArray[4]):
			htmlSTR += '<a title="' + commentsmsgs + '"> <img  src="/imgs/p_20.jpg" alt="Battry LOW" width=30 height=60" /></a>&nbsp;'
		else:
			htmlSTR += '<a title="' + commentsmsgs + '"> <img  src="/imgs/p_0.jpg" alt="Battry Very Low" width=30 height=60" /></a>&nbsp;'

			
	if(len(result) > 21):
		if("wlan0" in result):
			htmlSTR += '<a title="' + result.replace('"', '') + '"> <img  src="/imgs/wireless_ok.png" alt="wireless ok" width=30 height=30" /></a>'
		else:
			htmlSTR += '<a title="' + result.replace('"', '') + '"> <img  src="/imgs/wireless_ok_blue.png" alt="wireless ok" width=30 height=30" /></a>'
	else:
		htmlSTR += '<a title="' + result.replace('"', '') + '"> <img  src="/imgs/wireless_notok.png" alt="wireless notok" width=30 height=30" />'
				
	htmlSTR += '&nbsp;<a href="/">Home</a>&nbsp;|&nbsp;<a href="/terminal">Terminal</a>&nbsp;&nbsp;'
	htmlSTR += '<a href="/show_configs?filter=*">Config</a>&nbsp;&nbsp;'
	htmlSTR += '<a href="/imu">IMU</a>&nbsp;&nbsp;'
	htmlSTR += '<a href="/common_panel?fast=no">Panel</a>&nbsp;|&nbsp;'
	htmlSTR += '<a href="/cp">CP</a>&nbsp;&nbsp;'
	for idx in range(0, MAXNUMBER, 1):
		if not "dontshow_" in CQC[idx][1]:
			htmlSTR += '<a href="/launch?id=' + str(idx) + '&refresh=6&showmenu=true">' + CQC[idx][1] + ('</a>&nbsp;|&nbsp;' if idx ==3  else '</a>&nbsp;&nbsp;')
	htmlSTR += '|&nbsp;<a href="/logout-now">Logout (' + GetUsername() + ')</a></h3><hr/>'
	return htmlSTR

def menuBarWithConfig():
	return menuBar() + configBar()

class index(object):
	def GET(self, fileName):
		if("logout-now" in fileName):
			web.ctx.env['HTTP_AUTHORIZATION'] = None
			web.ctx.status = '401 Unauthorized'
			time.sleep(1)
			return '<font size=5px color="red">You successfully logged out</font>'
		global lastAccessTime
		global image_sub
		global head_pub
		global buz_pub
		global MAXNUMBER
		global rCounter
		global jpeg
		global headingData
		global headingDataRadian
		global locData
		global gpan
		global gtilt
		global lastDiagnostics
		global diagnosticsData
		global lastParam
		global paramData
		global A 
		global B 
		global E 
		global F 
		global G 
		global H
		global D 
		global I
		global scale
		global marginW
		global marginH
		global imgE
		global imgF
		global imgD
		global Width
		global Height
		global Width2
		global Height2
		global Radius
		global locW
		global locH
		global P1
		global P2
		global P3
		global P4
		global P5
		global P6
		global P7
		global P8
		global P9
		global P10
		global PO7
		global PO8
		global PO9
		global PO10
		global P11
		global P12
		global PO11
		global PO12
		global wifiConfigAddress
		global wifiAddress
		auth = web.ctx.env.get('HTTP_AUTHORIZATION')
		allow = False
		if  auth is not None:
			auth = re.sub('^Basic ', '', auth)
			username, password = base64.decodestring(auth).split(':')
			if (username, password) in allowed or web.ctx.ip == "127.0.0.1":
				allow = True
		if not allow:
			web.header('WWW-Authenticate', 'Basic realm="Auth example"')
			web.ctx.status = '401 Unauthorized'
			return
		if allow:
			pass
			# with open('loginLog.txt', 'a') as file:
			# 	file.write(time.strftime("%Y/%m/%d %H:%M:%S")+"  "+GetUsername()+ "  "+fileName+ "\r\n")	
    		global formp
    		lastAccessTime=datetime.datetime.now()
    		if ".jpg" in fileName or ".gif" in fileName or ".png" in fileName or ".tga" in fileName or ".bmp" in fileName: 
	      		web.header("Content-Type", "images/jpeg")  # Set the Header
	    	   	return open(fileName, "rb").read()  # Notice 'rb' for reading images
	    	elif ".yaml" in fileName :
	    		pathToConfig = ""
	    		try:
	    			pathToConfig = subprocess.check_output("rosparam get /config_server/config_path", shell=True).strip()
	    		except:
	    			pass
	      		web.header("Content-Type", "text/yaml")  # Set the Header
	    	   	return open(pathToConfig + "/" + fileName, "rb").read()  # Notice 'rb' for reading images
	    	elif fileName.startswith('setconfigparam') :
			user_data = web.input(value="", name="")
			result = ""
			if(len(user_data.name) > 3 and len(user_data.value) > 0):
				try:
					result = subprocess.check_output('rosservice call /config_server/set_parameter "{name: \'' + user_data.name + '\', value: \'' + user_data.value + '\', no_notify: \'0\'}"', shell=True)
			    	except:
			    		pass	
                	return result
	    	elif fileName.startswith('commandpan') :
			user_data = web.input(z="0")
			headMsg = LookAtTarget()
			headMsg.enabled = True
			headMsg.is_relative = False
			headMsg.is_angular_data = True
			headMsg.vec.x = 0
			headMsg.vec.y = gtilt
			headMsg.vec.z = float(user_data.z)
			headMsg.pitchEffort = 0.25
			headMsg.yawEffort = 0.25
			head_pub.publish(headMsg)
			gpan = float(user_data.z)
                	return ""
                elif fileName.startswith('commandtilt') :
			user_data = web.input(y="0")
			headMsg = LookAtTarget()
			headMsg.enabled = True
			headMsg.is_relative = False
			headMsg.is_angular_data = True
			headMsg.vec.x = 0
			headMsg.vec.y = float(user_data.y)
			headMsg.vec.z = gpan
			headMsg.pitchEffort = 0.25
			headMsg.yawEffort = 0.25
			head_pub.publish(headMsg)
			gtilt = float(user_data.y)
                	return ""
                elif fileName.startswith('commandgait') :
			user_data = web.input(x="0", y="0", z="0", walk="false",motion="0")
			gaitMsg=GaitCommand()
			gaitMsg.gcvX=float(user_data.x);
			gaitMsg.gcvY=float(user_data.y);
			gaitMsg.gcvZ=float(user_data.z);
			gaitMsg.walk=user_data.walk.lower() =="true";
			gaitMsg.motion=int(user_data.motion);
			if(CQ[2] is not None):#behaviour is running
				gait_beh_pub.publish(gaitMsg)
			else:
				gait_pub.publish(gaitMsg)
                	return ""
                elif fileName.startswith('setlocation') :
			user_data = web.input(x="0", y="0")
			posMsg = localization_input()
			posMsg.position.x = float(user_data.x)
			posMsg.position.y = float(user_data.y)
                        posMsg.setOrientation=False
                        posMsg.setPosition = True
			pos_pub.publish(posMsg)
                	return ""
                elif fileName.startswith('setorientation') :
			user_data = web.input(phi="0")
			posMsg = localization_input()
			posMsg.position.z = (np.sign(float(user_data.phi))*math.pi/2.)+locData[3]
                        posMsg.setOrientation=True
                        posMsg.setPosition = False
			pos_pub.publish(posMsg)
                	return ""
                elif fileName.startswith('terminal') :
                	return '''<!doctype html><html><head><script type="text/javascript" src="/static/jquery.js"></script></head>''' + '<body>' + menuBarWithConfig() + '''<iframe style="overflow: hidden;height:700px;width:100%" src="''' + web.ctx.home.replace("8080", "8081") + '''"><p>Your browser does not support iframes.</p></iframe>''' + '</div></div></body></html>'
    	 	elif fileName.startswith('common_panel') :	
    	 		user_data = web.input(fast="")
    	 		global lastVisionFast
    	 		imgcontent = ''
    	 		isFast = "yes" in user_data.fast or "true" in user_data.fast;
    	 		if(isFast):
    	 			imgcontent = 'id="img" width="320" height="240" src="' + web.ctx.home.replace(":8080", ":5000") + '/video_vision"'
    	 		else:
    	 			imgcontent = 'id="img" width="320" height="240" src="/video_feed'
    	 		
                	return '''<!doctype html><html><head><script type="text/javascript" src="/static/jquery.js"></script>   
				<style type="text/css">
				.kol { overflow: hidden; }
				.column { float: left; }
				.half1 { width: auto; }
				.last { float: none; width: auto; }
				</style>
					''' + '''<script type="text/javascript">
				$(function() {
				$("#imgPanel").click(function(e) {
				  var offset = $(this).offset();
				  var relativeX = (e.pageX - offset.left);
				  var relativeY = (e.pageY - offset.top);
				  alert(relativeX+':'+relativeY);
				  $(".position").val("afaf");
				});
				});
				function loadNowPlayingLOC() {jQuery("#boxLOC").load("/show_localization");}
				setInterval(function() {loadNowPlayingLOC()}, ''' + ("300" if isFast else "700") + ''');
				</script>''' + ("" if isFast else '''<script type="text/javascript">
				function loadNowPlaying2(){
					var d = new Date();
					jQuery("#img").attr("src", "/video_feed"+"?dummy=" + d.getTime());
				}
				setInterval(function(){loadNowPlaying2()}, 500);
				</script>''') + '''
				</head>''' + '<body>' + menuBarWithConfig() + '<div class="kol"> <div id="lefto" class="column half1"> <font size=5px color="red">Pan</font>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;' + '''
				<input type="range" style="width: 320px; height: 20px;
				-webkit-appearance: slider-horizontal;
				writing-mode: bt-lr;" min="-1.3" max="1.3" value="0" step="0.1" oninput="showValue(this.value)" />
				<span id="range">0</span>
				<script type="text/javascript">
				function showValue(newValue)
				{
					newValue=-1*newValue;
					document.getElementById("range").innerHTML=newValue.toFixed(2);
					  var xmlhttp;
					  if (window.XMLHttpRequest) {
					    xmlhttp = new XMLHttpRequest();
					  } else {
					    // code for older browsers
					    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
					  }
					  xmlhttp.onreadystatechange = function() {
					    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
					      document.getElementById("demo").innerHTML =
					      xmlhttp.responseText;
					    }
					  };
					  xmlhttp.open("GET", "''' + web.ctx.home + '''/commandpan?z="+newValue, false);
					  xmlhttp.send();
				}
				function showValue2(newValue)
				{
					newValue=1-newValue;
					document.getElementById("range2").innerHTML=newValue.toFixed(2);
					  var xmlhttp;
					  if (window.XMLHttpRequest) {
					    xmlhttp = new XMLHttpRequest();
					  } else {
					    // code for older browsers
					    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
					  }
					  xmlhttp.onreadystatechange = function() {
					    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
					      document.getElementById("demo").innerHTML =
					      xmlhttp.responseText;
					    }
					  };
					  xmlhttp.open("GET", "''' + web.ctx.home + '''/commandtilt?y="+newValue, false);
					  xmlhttp.send();
				}
				function showRot(newValue)
				{
					newValue=1*newValue;
					document.getElementById("rangerot").innerHTML=newValue.toFixed(2);
					return walkPrev();
				}
				function showScale(newValue)
				{
					newValue=1*newValue;
					document.getElementById("rangescale").innerHTML=newValue.toFixed(2);
					return walkPrev();
				}
				function setDoit(doit)
				{
					document.getElementById("rangedoit").innerHTML=doit;
					return walkPrev();
				}
				function walkPrev()
				{
					return walk(parseFloat(document.getElementById("rangex").innerHTML),parseFloat(document.getElementById("rangey").innerHTML),"PREV");
				}
				function walk(xval,yval,comment)
				{
					doit=document.getElementById("rangedoit").innerHTML;
					
					if(doit == "true")
					{
						document.getElementById("rangeangle").style.color = "green";
					}
					else
					{
						document.getElementById("rangeangle").style.color = "gray";
					}
					
					if(comment!="PREV")
					{
						document.getElementById("rangeangle").innerHTML = comment;
					}
					
					scale=parseFloat(document.getElementById("rangescale").innerHTML).toFixed(2);
					zval=-1*parseFloat(document.getElementById("rangerot").innerHTML).toFixed(2);
					document.getElementById("rangex").innerHTML=xval.toFixed(2);
					document.getElementById("rangey").innerHTML=yval.toFixed(2);
					
					xval=xval*scale;
					yval=-1*yval*scale;
					  var xmlhttp;
					  if (window.XMLHttpRequest) {
					    xmlhttp = new XMLHttpRequest();
					  } else {
					    // code for older browsers
					    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
					  }
					  xmlhttp.onreadystatechange = function() {
					    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
					      document.getElementById("demo").innerHTML =
					      xmlhttp.responseText;
					    }
					  };
					  xmlhttp.open("GET", "''' + web.ctx.home + '''/commandgait?walk="+doit+"&x="+xval+"&y="+yval+"&z="+zval, false);
					  xmlhttp.send();
				}
				</script>''' + '<br><font size=5px color="red">Tilt</font>&nbsp;&nbsp;' + '''<input type="range" style="width: 20px; height: 240px;
				-webkit-appearance: slider-vertical;
				writing-mode: bt-lr;" orient="vertical" min="0" max="1" value="1" step="0.1" oninput="showValue2(this.value)" />
				<span id="range2">0</span>
				<script type="text/javascript">
				function loadXMLDoc(txt) 
				{
					  var xmlhttp;
					  if (window.XMLHttpRequest) {
					    xmlhttp = new XMLHttpRequest();
					  } else {
					    // code for older browsers
					    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
					  }
					  xmlhttp.onreadystatechange = function() {
					    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
					      document.getElementById("demo").innerHTML =
					      xmlhttp.responseText;
					    }
					  };
					  xmlhttp.open("GET", "''' + web.ctx.home + '''/callservice?id="+txt, false);
					  xmlhttp.send();
					  alert(xmlhttp.responseText,txt=='9');
				}
		
			
			
				</script>
				'''+ '<a id="imgPanel"><img ' + imgcontent + ' alt="Vision" ></a>' + '''<a name='end'>&nbsp;</a><br>&nbsp;&nbsp;&nbsp;&nbsp;<div id="boxLOC" class="box" style="margin-left:42px"></div>'''+'''
				&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button onclick="setDoit('true')">Start Gait</button>&nbsp;&nbsp;&nbsp;&nbsp;<button onclick="setDoit('false')">Stop Gait</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;<button onclick="loadXMLDoc('15')">LeftKick</button>&nbsp;&nbsp;&nbsp;<button onclick="loadXMLDoc('14')">RightKick</button><br>'''  +'''
				<br>    
	&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<a id="myLink45" title="walk 45" href="#"
		onclick="walk(1,-1,'Left Forward');return false;"><img src="/imgs/45.png"
		alt="45"></a>
	<a id="myLink0" title="walk 0" href="#"
		onclick="walk(1,0,'Forward');return false;"><img src="/imgs/0.png"
		alt="0"></a>
	<a id="myLink315" title="walk 315" href="#"
		onclick="walk(1,1,'Right Forward');return false;"><img
		src="/imgs/315.png" alt="315"></a>&nbsp;<font size=3px color="red">Val:</font>&nbsp;&nbsp;<input type="range" style="width: 110px; height: 20px;
				-webkit-appearance: slider-horizontal;
				writing-mode: bt-lr;" orient="horizontal" min="0" max="1" value="0" step="0.1" oninput="showScale(this.value)" />
				<span id="rangescale">0</span>
	<br>

	&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<a id="myLink90" title="walk 90" href="#"
		onclick="walk(0,-1,'Left');return false;"><img src="/imgs/90.png"
		alt="90"></a>
	<a id="myLinkS" title="walk on the spot" href="#"
		onclick="walk(0,0,'On The Spot');return false;"><img src="/imgs/p_center.png"
		alt="Start"></a>
	<a id="myLink270" title="walk 270" href="#"
		onclick="walk(0,1,'Right');return false;"><img
		src="/imgs/270.png" alt="270"></a>&nbsp;<span id="rangeangle" style="color:gray;">On The Spot</span> <span id="rangex" style="display:none;">0</span> <span id="rangey" style="display:none;">0</span> <span id="rangedoit" style="display:none;">false</span>
	<br>

	&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<a id="myLink135" title="walk 135" href="#"
		onclick="walk(-1,-1,'Left Backward');return false;"><img
		src="/imgs/135.png" alt="135"></a>
	<a id="myLink180" title="service call 180" href="#"
		onclick="walk(-1,0,'Backward');return false;"><img
		src="/imgs/180.png" alt="180"></a>
	<a id="myLink225" title="walk 225" href="#"
		onclick="walk(-1,1,'Right Backward');return false;"><img
		src="/imgs/225.png" alt="225"></a> '''+'''<font size=3px color="red">Rot:</font>&nbsp;&nbsp;<input type="range" style="width: 110px; height: 20px;
				-webkit-appearance: slider-horizontal;
				writing-mode: bt-lr;" orient="horizontal" min="-1" max="1" value="0" step="0.1" oninput="showRot(this.value)" />
				<span id="rangerot">0</span> <br><br>
				 '''+'''</div>
				<div id="righto" class="column"><iframe style="height:900px;" scrolling="yes" width="1400px" src="''' + web.ctx.home + '''/show_configs?filter=/vision/|/localization/|/settings/|/walk_and_kick/|/cap_gait/&full=no">
				<p>Your browser does not support iframes.</p></iframe>''' + '</div></div>' + '</body></html>'
                elif fileName.startswith('show_heading') :
                	return '<body><font size=5px color="green">Heading = ' + str(round(headingData, 1)) + '</font> </body>'	      
                elif fileName.startswith('show_localization') :
                	PLOC = str(locData[0] * 10 * scale + Width2 + marginW) + ', ' + str(locData[1] * -10 * scale + Height2 + marginH)
			PLOC2 = str(locData[0] * 10 * scale + Width2 + marginW + 7 * math.cos(locData[2])) + ', ' + str(locData[1] * -10 * scale + Height2 + marginH - 7 * math.sin(locData[2]))
			PLOC3 = str(marginW) + ', ' + str(marginH)
			PLOC4 = str(marginW + 10 * math.cos(headingDataRadian)) + ', ' + str(marginH - 10 * math.sin(headingDataRadian))
                	return '''<!DOCTYPE HTML>
				<html><head><style>body {margin: 0px;padding: 0px;}</style></head><body>
				<canvas id="myCanvas" width="''' + str(locW) + '''" height="''' + str(locH) + '''"></canvas>
				    <script>
				      var canvas = document.getElementById('myCanvas');
				      var context = canvas.getContext('2d');
 				
				      context.beginPath();
				      context.moveTo(''' + P1 + ''');
				      context.lineTo(''' + P2 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 				
				      context.beginPath();
				      context.moveTo(''' + P2 + ''');
				      context.lineTo(''' + P3 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 				
				      context.beginPath();
				      context.moveTo(''' + P3 + ''');
				      context.lineTo(''' + P4 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 				
				      context.beginPath();
				      context.moveTo(''' + P4 + ''');
				      context.lineTo(''' + P1 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 				
 				
				      context.beginPath();
				      context.moveTo(''' + P5 + ''');
				      context.lineTo(''' + P6 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 
				      context.beginPath();
				      context.moveTo(''' + P7 + ''');
				      context.lineTo(''' + P8 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
				      context.beginPath();
				      context.moveTo(''' + P7 + ''');
				      context.lineTo(''' + P9 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
				      context.beginPath();
				      context.moveTo(''' + P8 + ''');
				      context.lineTo(''' + P10 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 
				      context.beginPath();
				      context.moveTo(''' + PO7 + ''');
				      context.lineTo(''' + PO8 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
				      context.beginPath();
				      context.moveTo(''' + PO7 + ''');
				      context.lineTo(''' + PO9 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
				      context.beginPath();
				      context.moveTo(''' + PO8 + ''');
				      context.lineTo(''' + PO10 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'green';
				      context.stroke();
 
				      context.beginPath();
				      context.arc(''' + P11 + ''', 4, 0, 2 * Math.PI, false);
				      context.lineWidth = 1;
				      context.fillStyle = 'blue';
      				      context.fill();
				      context.strokeStyle = 'blue';
				      context.stroke();
				      context.beginPath();
				      context.arc(''' + P12 + ''', 4, 0, 2 * Math.PI, false);
				      context.lineWidth = 1;
				      context.fillStyle = 'blue';
      				      context.fill();
				      context.strokeStyle = 'blue';
				      context.stroke();
				      context.beginPath();
				      context.arc(''' + PO11 + ''', 4, 0, 2 * Math.PI, false);
				      context.lineWidth = 1;
				      context.fillStyle = 'yellow';
      				      context.fill();
				      context.strokeStyle = 'yellow';
				      context.stroke();
				      context.beginPath();
				      context.arc(''' + PO12 + ''', 4, 0, 2 * Math.PI, false);
				      context.lineWidth = 1;
				      context.fillStyle = 'yellow';
      				      context.fill();
				      context.strokeStyle = 'yellow';
				      context.stroke();
 
 				
				      context.beginPath();
				      context.arc(''' + str(Width2 + marginW) + ''', ''' + str(Height2 + marginH) + ''', ''' + str(Radius) + ''', 0, 2 * Math.PI, false);
				      context.lineWidth = 1;
				      context.strokeStyle = 'green';
				      context.stroke();
 				      
				      context.beginPath();
				      context.arc(''' + PLOC + ''', 4, 0, 2 * Math.PI, false);
				      context.lineWidth = 1;
				      context.strokeStyle = 'blue';
				      context.stroke();
 				      
				      context.beginPath();
				      context.moveTo(''' + PLOC + ''');
				      context.lineTo(''' + PLOC2 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'red';
				      context.stroke();
				      context.beginPath();
				      context.moveTo(''' + PLOC3 + ''');
				      context.lineTo(''' + PLOC4 + ''');
				      context.lineWidth = 1;
				      // set line color
				      context.strokeStyle = 'red';
				      context.stroke();
 				      
				      canvas.addEventListener('click', function(evt) {
				      var rect = canvas.getBoundingClientRect();
				      var context = canvas.getContext('2d');
					context.beginPath();
					context.arc(evt.clientX - rect.left, evt.clientY - rect.top, 1, 0, 2 * Math.PI, false);
					context.lineWidth = 1;
					context.strokeStyle = 'black';
					context.stroke();
					  var xmlhttp;
					  if (window.XMLHttpRequest) {
					    xmlhttp = new XMLHttpRequest();
					  } else {
					    // code for older browsers
					    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
					  }
					  xmlhttp.onreadystatechange = function() {
					    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
					      document.getElementById("demo").innerHTML =
					      xmlhttp.responseText;
					    }
					  };
					  if(evt.button === left)
					  {
					          var xSet=((evt.clientX - rect.left)-''' + str(marginW + Width2) + ''')/(10.0*''' + str(scale) + ''');
					          var ySet=((evt.clientY - rect.top)-''' + str(marginH + Height2) + ''')/(10.0*''' + str(-scale) + ''');
					          xmlhttp.open("GET", "''' + web.ctx.home + '''/setlocation?x="+xSet+"&y="+ySet, false);
					  }
					  else if(evt.button === right)
					  {
					          var phiSet=1;
                                                  xmlhttp.open("GET", "''' + web.ctx.home + '''/setorientation?phi="+phiSet, false);
					  }
					  xmlhttp.send();
 					  
				      }, false);
				      
				
				      </script></body></html>'''	      
		elif fileName.startswith('retreiveconfig') :
			user_data = web.input(username="", password="")
			result = ""
			resultOk = False
			username,password=GetUserPass()
			try:
				robotName = subprocess.check_output("hostname", shell=True).strip()
# 				txtC = "sshpass -p " + user_data.password + " ssh -o StrictHostKeyChecking=no -t " + user_data.username + "@" + web.ctx.ip + ' \'sudo -H -u ' + user_data.username + ' bash -c -i "DISPLAY=:0.0 xterm -e bash -c -i \\"nimbro config retrieve ' + robotName + '.local && read -p \'"\'"\'Press enter to continue...\'"\'"\' -n1 -s\\""\''
				txtC = "wget http://"+username+":"+password+"@" + web.ctx.ip+ ":8080/retrieve?robot="+robotName+".local"
	    			result = subprocess.check_output(txtC, shell=True)
	    			resultOk = True
	    		except Exception, e:
	    			return str(e)
	    			pass
	    		if(not resultOk):
				return '<font size=4px color="red">Terminal failed</font><br>' + "<br><br><br><br><br><br><div>" + '</div><br>'
 			
	    		return '<font size=4px color="black">Successful terminal open</font><br>' + "<br><br><br><br><br><br><did>" + '</div><br>' 
	    	elif fileName.startswith('deploy') :
			user_data = web.input(username="", password="")
			result = ""
			resultOk = False
			username,password=GetUserPass()
			try:
				robotName = subprocess.check_output("hostname", shell=True).strip()
# 				txtC = "sshpass -p " + user_data.password + " ssh -o StrictHostKeyChecking=no -t " + user_data.username + "@" + web.ctx.ip + ' \'sudo -H -u ' + user_data.username + ' bash -c -i "DISPLAY=:0.0 xterm -e bash -c -i \\"nimbro deploy ' + robotName + '.local && read -p \'"\'"\'Press enter to continue...\'"\'"\' -n1 -s\\""\''
				txtC = "wget http://"+username+":"+password+"@" + web.ctx.ip+ ":8080/deploy?robot="+robotName+".local"
		    		result = subprocess.check_output(txtC, shell=True)
	    			resultOk = True
	    		except Exception, e:
	    			return str(e)
	    			pass
	    		if(not resultOk):
				return '<font size=4px color="red">Terminal failed</font><br>' + "<br><br><br><br><br><br><div>" + '</div><br>'
	    		return '<font size=4px color="black">Successful terminal open</font><br>' + "<br><br><br><br><br><br><did>" + '</div><br>' 
                elif fileName.startswith('show_configs') :
                	user_data = web.input(filter="", full="")
                	filters=user_data.filter.split("|")
                	if(lastParam is None or paramData is None):
                		return '<body>' + ("" if "no" in user_data.full else menuBarWithConfig()) + '<font size=5px color="Red">No config_server is running</font>' + '</body>'
                	firstOpen = True
                	javacript = '<script type="text/javascript">'
                	javacript += '''function showValue(obj,nametxt,type){
					var newValue;
					if(type=="float")
					{
						newValue=1*obj.value;
						document.getElementById("S"+obj.id).innerHTML=newValue.toFixed(4);
					}
					else if(type=="int")
					{
						newValue=1*obj.value;
						document.getElementById("S"+obj.id).innerHTML=newValue;
					}
					else if(type=="bool")
					{
						newValue=obj.checked ? 1 :0 ;
						document.getElementById("S"+obj.id).innerHTML=obj.checked;
					}
					else
					{
						newValue=obj.value;
						document.getElementById("S"+obj.id).innerHTML=newValue.slice(0,10);
					}
				}
				function changeValue(obj,nametxt,type)
				{
					var newValue;
					if(type=="float")
					{
						newValue=1*obj.value;
						document.getElementById("S"+obj.id).innerHTML=newValue.toFixed(4);
					}
					else if(type=="int")
					{
						newValue=1*obj.value;
						document.getElementById("S"+obj.id).innerHTML=newValue;
					}
					else if(type=="bool")
					{
						newValue=obj.checked ? 1 :0 ;
						document.getElementById("S"+obj.id).innerHTML=obj.checked;
					}
					else
					{
						newValue=obj.value;
						document.getElementById("S"+obj.id).innerHTML=newValue.slice(0,10);;
					}
						  var xmlhttp;
					  if (window.XMLHttpRequest) {
					    xmlhttp = new XMLHttpRequest();
					  } else {
					    // code for older browsers
					    xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
					  }
					  xmlhttp.onreadystatechange = function() {
					    if (xmlhttp.readyState == 4 && xmlhttp.status == 200) {
					      document.getElementById("demo").innerHTML =
					      xmlhttp.responseText;
					    }
					  };
					  if(type=="float")
					  {
					 	xmlhttp.open("GET", "''' + web.ctx.home + '''/setconfigparam?name="+nametxt+"&value="+newValue.toFixed(4), false);
					  }
					  else
					  {
					 	xmlhttp.open("GET", "''' + web.ctx.home + '''/setconfigparam?name="+nametxt+"&value="+newValue, false);
					  }
					  xmlhttp.send();
				}'''
                	myBody = ''
                	last = ""
                	s = Stack()
			for item in paramData.parameters:
				if(item.type == ""):
					continue
				if(len(filters) > 0):
					itIsValid=False;
					for filter in filters:
						if filter=="*" or (len(filter)>0 and filter.lower() in item.name.lower()):
							itIsValid=True
							break
					if not itIsValid:
						continue	
 				curl = item.name.split("/")
				for i in range(1, len(curl)):
					if(i - 1 >= len(s)):
						if(firstOpen):
							firstOpen = False
							myBody += '<ul class="outline"> <li>\n'
						else:
							myBody += "<ul> <li>\n"
						myBody += curl[i] + "\n"
						s.push(curl[i])
						pass  # Push
					elif(curl[i] == s[i - 1]):
						pass
					elif(curl[i] != s[i - 1]):
						diff = len(s) - i + 1
						for k in range(diff):
							if(k == diff - 1):
								myBody += ""
							else:
						        	myBody += "</li></ul>\n"
							s.pop()
						myBody += "<li>\n"
						myBody += curl[i] + "\n"
						s.push(curl[i])
						pass  # Pop
				jsName = item.name.replace("/", "").replace("_", "").replace("-", "")
				magNum = 0
				charac = "="
				for kp in range(magNum - len(item.name)):
					myBody += charac
				myBody += "<div style=\"position:absolute; height: 20px; margin:-20px 250px;\">"
				if  item.type == "float":

					myBody += '<input id="range' + jsName + '" type="range" style="width: 300px; height: 20px;"  min="' + str(item.min) + '" max="' + str(item.max) + '" value="' + str(item.value) + '" step="' + str(item.step) + '" oninput="showValue(this,\'' + item.name + '\',\'float\')" onchange="changeValue(this,\'' + item.name + '\',\'float\')" />'
					myBody += '<span id="Srange' + jsName + '">' + "%.4f" % float(item.value) + '</span>'
					myBody += "<br>"
				elif  item.type == "int":
					myBody += ' <input id="range' + jsName + '" type="range" style="width: 300px; height: 20px;"  min="' + str(item.min) + '" max="' + str(item.max) + '" value="' + str(item.value) + '" step="' + str(item.step) + '" oninput="showValue(this,\'' + item.name + '\',\'int\')" onchange="changeValue(this,\'' + item.name + '\',\'int\')" />'
					myBody += '<span id="Srange' + jsName + '">' + str(item.value) + '</span>'
					myBody += "<br>"
				elif  item.type == "bool":
					checked = "checked" if item.value == "1" else ""
					myBody += ' <input id="range' + jsName + '" type="checkbox" onclick="changeValue(this,\'' + item.name + '\',\'bool\')" ' + checked + '/>'
					myBody += '<span id="Srange' + jsName + '">' + ("true" if item.value == "1" else "false") + '</span>'
					myBody += "<br>"
				elif  item.type == "string":
					myBody += ' <input id="range' + jsName + '" type="text" style="width: 300px; height: 20px;"   value="' + str(item.value) + '"  onChange="changeValue(this,\'' + item.name + '\',\'string\')" />'
					myBody += '<span id="Srange' + jsName + '">' + str(item.value)[0:10] + '</span>'
					myBody += "<br>"
 				else:
 					myBody += "*"
 					myBody += "<br>"
 					
 				myBody += "</div>"
 				
			for sp in range(len(s)):
				myBody += "</li></ul>\n"
			javacript += "</script>"
			
			return '''<!doctype html><html> <head><script type="text/javascript" src="/static/jquery.js"></script><script  src="/static/outline.js"></script><style type="text/css">
				.outline {list-style: none;}
				.outline ul {list-style: none;}
				.outline li {cursor: auto;margin:4px 0px;}
				.olink {border-style: none;padding-right: 10px;}
				.oimg {border-style: none;}
				</style></head></script><body onload="outlineInit()"> ''' + ("" if "no" in user_data.full else menuBarWithConfig()) + '<br>' + myBody + javacript + '</body>'
		elif fileName.startswith('video_feed') :
			web.header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
			web.header("Content-Type", "image/jpeg")  # Set the Header
	    	   	return getjpeg()
	    	elif fileName.startswith('imu') :
			if(lastParam is None or paramData is None):
				return '<body>' + menuBarWithConfig() + '<font size=5px color="Red">No Robotcontrol is running</font>' + '</body>'
			return render.imu(menuBarWithConfig(), web.ctx.home.replace(":8080", "").replace("http://", ""), formp)
	    	elif fileName.startswith('cp') :
			return render.cp(menuBarWithConfig(), web.ctx.home.replace(":8080", "").replace("http://", ""), formp)
		elif fileName.startswith('launch') :
			user_data = web.input(id="", refresh="3", showmenu="true")
			refresh = 5
			menutxt = ""
			if(user_data.showmenu == "true"):
				menutxt = menuBar()
			if user_data.refresh is not None and user_data.refresh.isdigit():
				refresh = float(user_data.refresh)
			if not user_data.id.isdigit() or int(user_data.id) > MAXNUMBER:
				return '<body>' + menutxt + '<font size=5px color="Red">ID requested is not valid </font>' + '</body>'
			msgsc = 'onclick="window.location.href=\'/start?id=' + user_data.id + "&showmenu=" + user_data.showmenu + "\';\""
			if((CQ[int(user_data.id)] is None or CQ[int(user_data.id)].killed or True) and "robotcontrol" in CQC[int(user_data.id)][1].lower()):
				msgsc = 'onclick="alert(\'The robot might collapse!\');' + ";\""
			elif((CQ[int(user_data.id)] is None or CQ[int(user_data.id)].killed or True) and "resetall" in CQC[int(user_data.id)][1].lower()):
				msgsc = 'onclick="alert(\'Due to resetall, the robot might collapse!\');' + ";\""
			elif((CQ[int(user_data.id)] is None or CQ[int(user_data.id)].killed) and "reboot" in CQC[int(user_data.id)][1].lower()):
				msgsc = 'onclick="alert(\'The robot will restart and after restart it might collapse!\');' + ";\""				
			elif((CQ[int(user_data.id)] is None or CQ[int(user_data.id)].killed) and "poweroff" in CQC[int(user_data.id)][1].lower()):
				msgsc = 'onclick="alert(\'The robot will shutdown!\');' + ";\""		
			return render.launch(menutxt, user_data.id, user_data.showmenu, CQC[int(user_data.id)][1], refresh * 100, msgsc)	
		elif("callservice" in fileName):
			user_data = web.input(id="")
			result = ""
			try:
				serverIP=""
				serverIP2=""
				serviceName = ""
				if "1" == user_data.id:
					subprocess.check_output('rosservice call /config_server/set_parameter "{name: \'/fallProtection/fallProtectionEnabled\', value: \'0\', no_notify: \'0\'}"', shell=True)
					serviceName = "/nimbro_op_interface/magFilter/startCalibration"
				elif "2" == user_data.id:
					subprocess.check_output('rosservice call /config_server/set_parameter "{name: \'/fallProtection/fallProtectionEnabled\', value: \'1\', no_notify: \'0\'}"', shell=True)
					serviceName = "/nimbro_op_interface/magFilter/stopCalibration2D"
				elif "3" == user_data.id:
					subprocess.check_output('rosservice call /config_server/set_parameter "{name: \'/fallProtection/fallProtectionEnabled\', value: \'1\', no_notify: \'0\'}"', shell=True)
					serviceName = "/nimbro_op_interface/magFilter/stopCalibration3D"
				elif "4" == user_data.id:
					serviceName = "/nimbro_op_interface/attEstCalibrate"
				elif "5_" in user_data.id:
					serviceName = "/nimbro_op_interface/magFilter/warpAddPoint " + user_data.id.replace("5_", "")
				elif "6" == user_data.id:
					serviceName = "/nimbro_op_interface/magFilter/showCalibration"
				elif "7" == user_data.id:
					serviceName = "/nimbro_op_interface/magFilter/warpClearPoints"
				elif "8" == user_data.id:
					robotName = subprocess.check_output("rosparam get /config_server/robot_name", shell=True)
					serviceName = '/config_server/save "filename: \'config_' + robotName.strip() + '\'"'
				elif "9" == user_data.id:
					robotName = subprocess.check_output("rosparam get /config_server/robot_name", shell=True)
					serviceName = '/config_server/load "filename: \'config_' + robotName.strip() + '\'"'
				elif "10" == user_data.id:
					serviceName = '/nimbro_op_interface/calibrateGyroStart "{type: 2, turns: 2}"'
				elif "11" == user_data.id:
					serviceName = '/nimbro_op_interface/calibrateGyroStart "{type: 1, turns: 2}"'
				elif "12" == user_data.id:
					serviceName = '/nimbro_op_interface/calibrateGyroReturn'
				elif "13" == user_data.id:
					serviceName = '/nimbro_op_interface/calibrateGyroStop'
				elif "14" == user_data.id:
					serviceName = '/motion_player/play "{name: \'kick_right\', type: 0}"'
				elif "15" == user_data.id:
					serviceName = '/motion_player/play "{name: \'kick_left\', type: 0}"'
				elif "16" == user_data.id:
					serviceName = '/motion_player/reload'
				elif "17" == user_data.id:
					serverIP = subprocess.check_output("rosservice call /config_server/get_parameter \"name: '/game_controller/serverIP'\"", shell=True)
					serviceName = '/config_server/set_parameter "{name: \'/game_controller/useLastServerIP\', value: \'1\', no_notify: \'0\'}"'
				elif "18" == user_data.id or "19" ==  user_data.id or "20" ==  user_data.id:
					conv = Ansi2HTMLConverter()
					sudoPassword = 'nimbro'
					if("18" ==  user_data.id):
						command = 'sudo python '+wifiAddress+' -f '+wifiConfigAddress
					elif("19" ==  user_data.id):
						command = 'sudo python '+wifiAddress+' -f '+wifiConfigAddress+" -b"
					else:
						command = 'sudo python '+wifiAddress+' -f '+wifiConfigAddress+" -b ; sleep 0.3;"+'sudo python '+wifiAddress+' -f '+wifiConfigAddress		
					result = subprocess.check_output('echo %s|sudo -S %s' % (sudoPassword, command), shell=True)
					if("successfully" in result) or "19" ==  user_data.id:
						return '<font size=4px color="green">Wifi command Successfull</font><br>' + serviceName + "<br><br><br><br><br><br>" + conv.convert(result) + '<br>'
					else:
						return '<font size=4px color="red">Wifi command failed </font><br>' + serviceName + "<br><br><br><br><br><br>" + conv.convert(result) + '<br>'
				else:
					return '<font size=4px color="red">Requested service is not found!</font>'
				
				result = subprocess.check_output("rosservice call " + serviceName, shell=True)
				result = result.replace("\n", "<br>")
				buzMsg = Buzzer()
				buzMsg.soundType = 1
				buzMsg.musicIndex = 21
				buz_pub.publish(buzMsg)
				
				if "17" == user_data.id:
					serverIP2 = subprocess.check_output("rosservice call /config_server/get_parameter \"name: '/game_controller/serverIP'\"", shell=True)
					return '<font size=4px color="green">Successful service call </font><br> Set GC IP' + "<br>Server IP was: "+serverIP+"<br>Server IP is now: "+serverIP2+"<br><br><br>" + result + '<br>'
				
				return '<font size=4px color="green">Successful service call </font><br>' + serviceName + "<br><br><br><br><br><br>" + result + '<br>'
			except:
				buzMsg = Buzzer()
				buzMsg.soundType = 1
				buzMsg.musicIndex = 8
				buz_pub.publish(buzMsg)
				return '<font size=4px color="red">Service call failed </font><br>' + serviceName + "<br><br><br><br><br><br>" + result + '<br>'
			
		elif("partial" in fileName):
			user_data = web.input(id="")
			if not user_data.id.isdigit() or int(user_data.id) > MAXNUMBER:
				return '<body>' + menuBar() + '<font size=5px color="Red">ID requested is not valid </font>' + '</body>'
			if(CQ[int(user_data.id)] is None):
				return '<font size=5px color="Red">' + CQC[int(user_data.id)][1] + ' is not started yet </font>'
			
			if("top" == CQC[int(user_data.id)][1].lower()):
				CQ[int(user_data.id)].outputG = CQ[int(user_data.id)].outputG.split("top - ")[-1]
			# The idea is to call Ansi2HTMLConverter just once  
			if(CQ[int(user_data.id)].outputGCounter % 2 == 0):
				conv = Ansi2HTMLConverter()
				CQ[int(user_data.id)].outputGHtml = conv.convert(CQ[int(user_data.id)].outputG)
				CQ[int(user_data.id)].outputGCounter += 1
				
			return CQ[int(user_data.id)].outputGHtml + '<br><font size=4px color="green">' + str(datetime.datetime.now())[:-4] + '</font>' + "<a name='end'>&nbsp;</a>" + '</body>'
		elif("fulloutput" in fileName):
			user_data = web.input(id="")
			if not user_data.id.isdigit() or int(user_data.id) > MAXNUMBER:
				return '<body>' + menuBarWithConfig() + '<font size=5px color="Red">ID requested is not valid </font>' + '</body>'
			if(CQ[int(user_data.id)] is None):
				return '<font size=5px color="Red">' + CQC[int(user_data.id)][1] + ' is not started yet </font>'
			# The idea is to call Ansi2HTMLConverter just once  
			if(CQ[int(user_data.id)].outputTotalGCounter % 2 == 0):
				conv = Ansi2HTMLConverter()
				CQ[int(user_data.id)].outputTotalGHtml = conv.convert(CQ[int(user_data.id)].outputTotalG)
				CQ[int(user_data.id)].outputTotalGCounter += 1
				
			return CQ[int(user_data.id)].outputTotalGHtml + '<br><font size=4px color="green">' + str(datetime.datetime.now())[:-4] + '</font>' + "<a name='end'>&nbsp;</a>" + '</body>'
    		elif("start" in fileName):
			user_data = web.input(id="", showmenu="true")
			menutxt = ""
			if(user_data.showmenu == "true"):
				menutxt = menuBar()
			if not user_data.id.isdigit() or int(user_data.id) > MAXNUMBER:
				return '<body>' + menutxt + '<font size=5px color="Red">ID requested is not valid </font>' + '</body>'
			if(CQ[int(user_data.id)] is None):
				if("killall" == CQC[int(user_data.id)][1].lower()):
					toReturn = "These tasks were killed:"
					aliveC = 0
					for pr in range(len(CQ)):
						if(CQ[int(pr)] is not None):
							aliveC += 1
							CQ[int(pr)].kill()
							CQ[int(pr)] = None
							toReturn += "&nbsp;&nbsp;" + CQC[int(pr)][1] + ",";
					if(aliveC < 1):
						toReturn = "There are no tasks to kill."
					return '<body>' + menutxt + '<font size=5px color="green">' + toReturn + '</font></body>'
				if("resetall" == CQC[int(user_data.id)][1].lower()):
					toReturn = "These tasks were Restarted:"
					aliveC = 0
					for pr in range(len(CQ)):
						if(CQ[int(pr)] is not None):
							aliveC += 1
							CQ[int(pr)].kill()
							CQ[int(pr)] = None
							CQ[int(pr)] = ThreadedCommand(CQC[pr][0], CQC[pr][3], CQC[pr][4], False, CQC[pr][2])
							toReturn += "&nbsp;&nbsp;" + CQC[int(pr)][1] + ",";
					if(aliveC < 1):
						toReturn = "There are no tasks to restart."
					return '<body>' + menutxt + '<font size=5px color="orange">' + toReturn + '</font></body>'
				else:
					CQ[int(user_data.id)] = ThreadedCommand(CQC[int(user_data.id)][0], CQC[int(user_data.id)][3], CQC[int(user_data.id)][4], False, CQC[int(user_data.id)][2])
				
			raise web.seeother("launch?id=" + user_data.id + "&showmenu=" + user_data.showmenu)
    		elif("killall" in fileName):
			user_data = web.input(id="", showmenu="true")
			menutxt = ""
			if(user_data.showmenu == "true"):
				menutxt = menuBar()
			if not user_data.id.isdigit() or int(user_data.id) > MAXNUMBER:
				return '<body>' + menutxt + '<font size=5px color="Red">ID requested is not valid </font>' + '</body>'
			if(CQ[int(user_data.id)] is not None):
				CQ[int(user_data.id)].kill()
				CQ[int(user_data.id)] = None
			raise web.seeother("launch?id=" + user_data.id + "&showmenu=" + user_data.showmenu)
		else:
			resultwifi = "NO WIFI"
			resulthost = "XS?"
			resultanalyse = "Please run Robotcontrol to see diagnostics!"
			try:
				resulthost = subprocess.check_output("hostname", shell=True)
				ifcRes=subprocess.check_output("ifconfig",shell=True)
				regEx=re.search(r'wlan\d',ifcRes.rstrip())
				if regEx:
				  	resultwifi = subprocess.check_output("iwgetid "+regEx.group(0), shell=True)
				if controlIsAlive():
					resultanalyse = "batteryVoltage" + str(diagnosticsData).replace("\n", "<br>").split("batteryVoltage")[-1]
    			except:
		  		pass
			myBody = '<font size=5px color="orange">' + resulthost + '</font><br>' + '<font size=5px color="black">' + resultwifi + '</font><hr>' + '<font size=5px color="black">' + resultanalyse + '</font>'
			return configBarPage(myBody)
			
	    	
	def POST(self, fileName):
	    	global formp
	        auth = web.ctx.env.get('HTTP_AUTHORIZATION')
	        allow = False
	        if  auth is not None:
	                auth = re.sub('^Basic ', '', auth)
	                username, password = base64.decodestring(auth).split(':')
	                if (username, password) in allowed:
	                        allow = True
	        if not allow:
	                web.header('WWW-Authenticate', 'Basic realm="Auth example"')
	                web.ctx.status = '401 Unauthorized'
	                return
	        if allow:
	        	return "POST: Not ready yet!"

if __name__ == "__main__":
	signal.signal(signal.SIGINT, signal_handler)
	thread = threading.Thread(target=runFlask, args=())
        thread.daemon = True  # Daemonize thread
        thread.start()
        web.config.debug = False
	app = web.application(urls, globals())
	app.internalerror = web.debugerror
	app.run()
	os.abort()