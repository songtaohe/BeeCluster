from model import SegmentationModel
from dataloader import SegmentationDataLoader

import numpy as np 

import tensorflow as tf 

from PIL import Image 

log_folder = "alllogs"
run = "run5"

image_size = 224
batchsize = 32


with tf.Session(config=tf.ConfigProto()) as sess:
	model = SegmentationModel(sess, image_size=image_size)

	writer = tf.summary.FileWriter(log_folder+"/"+run, sess.graph)

	dataloader_train = SegmentationDataLoader("dataset/all/")
	dataloader_train.preload(num=256, image_size=image_size)

	dataloader_test = SegmentationDataLoader("dataset/all/")
	dataloader_test.preload(num=128, image_size=image_size)


	step = 0 
	lr = 0.001
	sum_loss = 0 

	while True:

		sat,gt = dataloader_train.getBatch(batchsize)

		loss,_ = model.Train(sat,gt,lr)

		sum_loss += loss 

		if step > 0 and step % 100 == 0:
			sum_loss /= 100
			
			test_loss = 0

			for j in range(128/batchsize):
				sat, gt = dataloader_test.getBatch(batchsize, st=j*batchsize)
				loss, output = model.Evaluate(sat,gt)
				test_loss += loss 

				if step % 100 == 0:
					for k in range(batchsize):
						sat_img = ((sat[k,:,:,:]+0.5) * 255.0).reshape((image_size,image_size,3)).astype(np.uint8)
						output_img = (output[k,:,:,0] * 255.0).reshape((image_size/4,image_size/4)).astype(np.uint8)
						gt_img = (gt[k,:,:,0] * 255.0).reshape((image_size/4,image_size/4)).astype(np.uint8)

						Image.fromarray(sat_img).save("validation/tile%d_sat.png" % (j*batchsize+k))
						Image.fromarray(output_img).save("validation/tile%d_output.png" % (j*batchsize+k))
						Image.fromarray(gt_img).save("validation/tile%d_gt.png" % (j*batchsize+k))

			test_loss /= (128/batchsize)

			print("step", step, "loss", sum_loss, "test loss", test_loss)


			summary = model.addLog(test_loss, sum_loss)
			writer.add_summary(summary, step)
				

		if step > 0 and step % 100 == 0:
			dataloader_train.preload(num=256, image_size=image_size)


		if step > 0 and step % 5000 == 0:
			model.saveModel("model20190919_2/model%d" % step)


		if step > 0 and step % 5000 == 0:
			lr = lr / 3

		step += 1
