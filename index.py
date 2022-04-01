#-*- coding:utf-8 -*-
from flask import Flask, request
from flask import render_template
import RPi.GPIO as GPIO
import time			#미세먼지
import Adafruit_DHT
import numpy as np
import cv2
import spidev #spi 라이브러리를 불러온다.
import os		# 미세먼지 관련 라이브러리
import fcntl	# 미세먼지 관련 라이브러리

#--------------------------------------
#LED
app=Flask(__name__)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

led_pin1=26						#LED를 control할 GPIO 번호
led_pin2=14
led_pin3=15
GPIO.setup(led_pin1,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(led_pin2,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(led_pin3,GPIO.OUT,initial=GPIO.LOW)

#--------------------------
#Temperature and Humidity
sensor = Adafruit_DHT.DHT11
pin = 24
#--------------------------------------
#cctv
#work 디렉토리의 haarcascade_frontalface_default.xml 파일을 Classifier로 사용
faceCascade = cv2.CascadeClassifier('/home/pi/Smart_Home/home/haarcascade_frontalface_default.xml')
eyeCascade = cv2.CascadeClassifier('/home/pi/Smart_Home/cctv/work/haarcascade_eye.xml')

cap = cv2.VideoCapture("http://192.168.0.201:8091/?action=stream")
cap.set(3,640) # set Width
cap.set(4,480) # set Height
#--------------------------------------
#gas
#SPI 인스턴스 spi생성
spi = spidev.SpiDev()

#라즈베리파이와 SPI통신 시작하기
spi.open(0,0)	#open(bus, device)

#SPI 통신 속도 설정
spi.max_speed_hz = 100000

#딜레이 시간(센서 측정 간격)
delay = 2

#MCP3008채널 중 센서에 연결한 채널 설정
pot_channel = 0

#-------------------------------------
#dust
I2C_SLAVE = 0x703
PM2008 = 0x28

fd = os.open('/dev/i2c-1',os.O_RDWR)
if fd < 0 :
	print("Failed to open the i2c bus\n")
io = fcntl.ioctl(fd,I2C_SLAVE,PM2008)
if io < 0:
	print("Failed to acquire bus access/or talk to salve\n")
#-------------------------------------	
	
@app.route("/")
def home():
	return render_template("index.html")
	
@app.route("/camera")
def camera():
	ix =0	
	while True:
		
		ret, img = cap.read()
		img = cv2.flip(img, 1) # 상하반전
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		faces = faceCascade.detectMultiScale(
			gray,
			scaleFactor=1.2,
			minNeighbors=5,
			minSize=(20,20)
		)
		for (x,y,w,h) in faces:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			roi_gray = gray[y:y+h, x:x+w]
			roi_color = img[y:y+h, x:x+w]
			cv2.imwrite('/home/pi/Smart_Home/home/face/face.'+str(ix)+'.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,90])
			ix +=1	
		K = cv2.waitKey(30) & 0xff
		if K == 27: # press 'ESC' to quit # ESC를 누르면 종료
			break
		
		#cv2.imwrite('/home/pi/Smart_Home/cctv/face/face.'+str(ix)+'.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,90])
		#cv2.imwrite('/home/pi/Smart_Home/cctv/face/face_{ix:04d}.jpg', img, [cv2.IMWRITE_JPEG_QUALITY,90])		
		cv2.imshow('video',img) # video라는 이름으로 출력력
	cap.release()
	cv2.destroyAllWindows()
	return "ok"
	
#거실 전등
@app.route("/led1/on")
def led1_on():
	try:
		GPIO.output(led_pin1, GPIO.HIGH)
		return "ok"
	except expression as identifier:
		return "fail"
		
@app.route("/led1/off")
def led1_off():
	try:
		GPIO.output(led_pin1, GPIO.LOW)
		return "ok"
	except expression as identifier:
		return "fail"
		
@app.route("/led1/check")
def led1_check():
	try:
		check1 = format(GPIO.input(26))
		return check1
	except expression as identifier:
		return "fail"

#화장실 전등
@app.route("/led2/on")
def led2_on():
	try:
		GPIO.output(led_pin2, GPIO.HIGH)
		return "ok"
	except expression as identifier:
		return "fail"
		
@app.route("/led2/off")
def led2_off():
	try:
		GPIO.output(led_pin2, GPIO.LOW)
		return "ok"
	except expression as identifier:
		return "fail"

@app.route("/led2/check")
def led2_check():
	try:
		check2 = format(GPIO.input(14))
		return check2
	except expression as identifier:
		return "fail"
		
#방 전등
@app.route("/led3/on")
def led3_on():
	try:
		GPIO.output(led_pin3, GPIO.HIGH)
		return "ok"
	except expression as identifier:
		return "fail"
		
@app.route("/led3/off")
def led3_off():
	try:
		GPIO.output(led_pin3, GPIO.LOW)
		return "ok"
	except expression as identifier:
		return "fail"

@app.route("/led3/check")
def led3_check():
	try:
		check3 = format(GPIO.input(15))
		return check3
	except expression as identifier:
		return "fail"


@app.route("/Humid")
def Humid():
	try:
		#while True:
		h, t = Adafruit_DHT.read_retry(sensor, pin)
		print("Temperature = {0:0.1f}*C",t)
		Humidity = "{1:0.1f}%".format(t,h)
		msg = "{0:0.1f} {1:0.1f}".format(t,h)
		#time.sleep(2)
		return msg
	except KeyboardInterrupt:
		print("Terminated by Keyboard")
		return "fail"

def analog_read(adcnum):

	if adcnum > 7 or adcnum < 0:
		return -1
	r = spi.xfer2([1,(8+adcnum)<<4,0])
	data = ((r[1]&3) << 8) + r[2]
	return data

@app.route("/gas")
def gas():
	gas = analog_read(pot_channel)
	print("------------------------")
	print("gas: %dppm" %(gas))
	msg = "{0}".format(gas)
	#time.sleep(2)
	return msg
	
@app.route("/dust")
def dust():
	try:
		#while True:
		data = os.read(fd,32)
		#print("Status=",int(data[2]),",MeasuringMode=",256*int(data[3])+int(data[4]),",CalivCoeff=",256*int(data[5])+int(data[6]),"\n")
		print("GRIM: PM2.5 = ", 256*int(data[9])+int(data[10]),",PM10 = ",256*int(data[11])+int(data[12]),"\n")
		print("--------------------------------------------------------------------------------")
		pm25 = 256*int(data[9])+int(data[10])
		pm10 = 256*int(data[11])+int(data[12])
		msg = "{0} {1}".format(pm25,pm10)
		#time.sleep(10)
		return msg
	except KeyboardInterrupt:
		#os.close(fd)
		print("Terminated by Keyboard")
		return "fail"		
#GPIO 설정 초기화(모듈 점유 해제)
@app.route("/gpio/cleanup")
def gpio_cleanup():
	GPIO.cleanup()
	return "GPIO CLENUP"
	
if __name__=="__main__":
	app.run(host="0.0.0.0",port=8888, threaded=True)



	



