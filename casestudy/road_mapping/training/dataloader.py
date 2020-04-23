import scipy 
import scipy.ndimage
import numpy as np 
import random 
import os 
from PIL import Image 


class SegmentationDataLoader():
	def __init__(self, folder):
		self.folder = folder
		self.filelist = []

		files = os.listdir(folder)

		for file in files:
			if file.endswith("in.jpg"):
				self.filelist.append(folder + "/" + file)

		print("dataset size", len(self.filelist))

		self.num = 0

	def preload(self, image_size=224, num = 1024):

		tiles = []

		self.num = num 
		self.sats = np.zeros((num, image_size, image_size, 3))
		self.gts = np.zeros((num, image_size/4, image_size/4, 2), dtype=np.int)


		for i in range(num):
			#if i % 10 == 0:
			#	print(i)

			ind = random.randint(0,len(self.filelist)-1)

			sat = scipy.ndimage.imread(self.filelist[ind])
			gt_img = scipy.ndimage.imread(self.filelist[ind].replace("in","gt"))
			sat = scipy.misc.imresize(sat, (480/2,640/2))
			gt_img = scipy.misc.imresize(gt_img, (480/8,640/8))


			# random rotation

			angle = random.randint(-45,45)

			sat = scipy.ndimage.interpolation.rotate(sat, angle, reshape=False, mode='nearest')
			gt_img = scipy.ndimage.interpolation.rotate(gt_img, angle, reshape=False)


			# random color 
			sat[:,:,0] = sat[:,:,0] * (min(1.0, random.random() + 0.5))
			sat[:,:,1] = sat[:,:,1] * (min(1.0, random.random() + 0.5))
			sat[:,:,2] = sat[:,:,2] * (min(1.0, random.random() + 0.5))

			exp = random.random()*0.8 + 0.8
			sat = np.clip(sat * exp, 0, 255)


			# normalize
			sat = (sat.astype(np.float) - 127.0) / 255.0

			gt = np.zeros((480/8,640/8,2), dtype =np.int)
			road = gt[:,:,0]
			nonroad = gt[:,:,1]

			road[np.where(gt_img>127)] = 1
			nonroad[np.where(gt_img<=127)] = 1

			gt[:,:,0] = road 
			gt[:,:,1] = nonroad 

			# todo argumentation 

			cx,cy = 120, 160 

			cx += random.randint(-6,6)
			cy += random.randint(-10,10)


			self.sats[i,:,:,:] = sat[cx-112:cx+112, cy-112:cy+112, :]

			self.gts[i,:,:,:] = gt[cx/4-112/4:cx/4+112/4, cy/4-112/4:cy/4+112/4]



	def getBatch(self, batchsize = 64, st = None):
		if st is None:
			st = random.randint(0, self.num-1-batchsize)

		return self.sats[st:st+batchsize,:,:,:], self.gts[st:st+batchsize,:,:,:]



if __name__ == "__main__":
	loader = SegmentationDataLoader("dataset/all/")

	loader.preload(224,16)


	for i in range(16):
		print(i)
		sat = ((loader.sats[i,:,:,:] + 0.5) * 255.0).astype(np.uint8)
		gt = ((loader.gts[i,:,:,0]) * 255.0).astype(np.uint8)

		Image.fromarray(sat.reshape((224,224,3))).save("tmp/img_%d_in.jpg" % i)
		Image.fromarray(gt.reshape((56,56))).save("tmp/img_%d_gt.jpg" % i)





