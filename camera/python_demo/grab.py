#coding=utf-8
from __future__ import print_function
from __future__ import division
import mvsdk
import time
import glob
import os
import sys
import cv2
import numpy as np
sys.path.append(r'..\..\insightface\deploy')
import face_model
# face_model里面有scipy 好像这个包会导致ctrl+c不可用 还没解决？

unfold = "D:/11projects/pal-face/camera/python_demo/ocamcalib_for_panoramic_unfold/Release/Project1 ./calib_results_1121.txt"

# os.chdir(os.getcwd())

def main():
	# 枚举相机
	DevList = mvsdk.CameraEnumerateDevice()
	nDev = len(DevList)
	if nDev < 1:
		print("No camera was found!")
		return

	for i, DevInfo in enumerate(DevList):
		print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
	i = 0 if nDev == 1 else int(input("Select camera: "))
	DevInfo = DevList[i]
	print(DevInfo)

	# 打开相机
	hCamera = 0
	try:
		hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
	except mvsdk.CameraException as e:
		print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
		return

	# 获取相机特性描述
	cap = mvsdk.CameraGetCapability(hCamera)
	# PrintCapbility(cap)

	# 判断是黑白相机还是彩色相机
	monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

	# 黑白相机让ISP直接输出MONO数据，而不是扩展成R=G=B的24位灰度
	if monoCamera:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
	else:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

	# 相机模式切换成连续采集
	mvsdk.CameraSetTriggerMode(hCamera, 0)

	# 手动曝光，曝光时间30ms
	mvsdk.CameraSetAeState(hCamera, 0)
	mvsdk.CameraSetExposureTime(hCamera, 100 * 1000)

	# 让SDK内部取图线程开始工作
	mvsdk.CameraPlay(hCamera)

	# 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
	FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

	# 分配RGB buffer，用来存放ISP输出的图像
	# 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
	pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

	savetime=0

	img_list = glob.glob("./img/*.jpg")
	if len(img_list) != 0:
		num_img = len(img_list)
	else:
		num_img = 0

	### 模型加载
	image_size = "112,112"
	model_path = "../../insightface/models/model-y1-test2/model,0"
	ga_model_path = "../../insightface/models/gamodel-r50/model,0"
	args = ReadArgs(image_size, model_path, ga_model_path)
	model = face_model.FaceModel(args)

	## 开始抓拍
	try:

		# this script can't be stopped by ctrl+c or quit automatically if met
		# with any error, probably because of some multi threads process.
		while(True):
			# print("ent loop")
			# 从相机取一帧图片
			try:
				# print("get buffer")
				# pass
				pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
				mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
				mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
				pFrameBuffer, piWidth, piHeight = mvsdk.CameraGetImageBufferEx(hCamera, 2000)

				# 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
				# 该示例中我们只是把图片保存到硬盘文件中
				if time.time()-savetime >= 1:
					# 存下一张图片
					img_name = "./img/grab_{}.jpg".format(num_img)
					status = mvsdk.CameraSaveImage(hCamera, img_name, pFrameBuffer, FrameHead, mvsdk.FILE_JPG, 100)
					if status == mvsdk.CAMERA_STATUS_SUCCESS:
						print("Save image successfully. image_size = {}X{}".format(FrameHead.iWidth, FrameHead.iHeight) )
					else:
						print("Save image failed. err={}".format(status) )

					# 调用cpp读入刚存的图片，展开和去畸变,最好把图像做参数直接传进去，把展开图直接传出来
					out_path = "./result/{}.jpg".format(num_img)
					os.system(unfold+" "+out_path+" "+img_name)
					print("Unfold image successfully.")

					# 调用insightface进行人脸检测,存下截取到的人脸图像
					raw_img = cv2.imread(out_path)
					try:
						face_img = model.get_input(raw_img)
						face_path = "./face/{}.jpg".format(num_img)
						# the channels of face_img is adapted to fit the network input
						# so we should use a np.transpose here
						face_img = np.transpose(face_img, [1,2,0])
						face_img = cv2.cvtColor(face_img, cv2.COLOR_BGR2RGB)
						cv2.imwrite(face_path, face_img)
						print("Face detected and saved successfully.")
					except:
						pass
					# 实时获取人脸特征向量，对比确认身份
					# feature = model.get_feature(face_img)


					# 将图像存入对应人的文件夹


					# status = mvsdk.CameraSaveImage(hCamera, img_name, pFrameBuffer, FrameHead, mvsdk.FILE_JPG, 100)
					# if status == mvsdk.CAMERA_STATUS_SUCCESS:
					# 	print("Save image successfully. image_size = {}X{}".format(FrameHead.iWidth, FrameHead.iHeight) )
					# else:
					# 	print("Save image failed. err={}".format(status) )
					savetime = time.time()
					num_img +=1
				else:
					continue
			except mvsdk.CameraException as e:
				print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )
	except KeyboardInterrupt:
		print("KeyboardInterrupt to exit.")
		# pass
		# 关闭相机
		mvsdk.CameraUnInit(hCamera)

		# 释放帧缓存
		# mvsdk.CameraAlignFree(pFrameBuffer)

def PrintCapbility(cap):
	for i in range(cap.iTriggerDesc):
		desc = cap.pTriggerDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iImageSizeDesc):
		desc = cap.pImageSizeDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iClrTempDesc):
		desc = cap.pClrTempDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iMediaTypeDesc):
		desc = cap.pMediaTypeDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iFrameSpeedDesc):
		desc = cap.pFrameSpeedDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iPackLenDesc):
		desc = cap.pPackLenDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iPresetLut):
		desc = cap.pPresetLutDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iAeAlmSwDesc):
		desc = cap.pAeAlmSwDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iAeAlmHdDesc):
		desc = cap.pAeAlmHdDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iBayerDecAlmSwDesc):
		desc = cap.pBayerDecAlmSwDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iBayerDecAlmHdDesc):
		desc = cap.pBayerDecAlmHdDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )


class ReadArgs():
	def __init__(self, image_size, model, ga_model, gpu=0, det=0, flip=0, threshold=1.24):
		self.image_size = image_size
		self.model = model
		self.ga_model = ga_model
		self.gpu = gpu
		self.det = det
		self.flip = flip
		self.threshold = threshold

main()
