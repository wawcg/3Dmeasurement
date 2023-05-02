import os
from PIL import Image

path = os.getcwd()

#对目录下的文件进行遍历
for file in os.listdir(path):
	if file.split('.')[-1] == "bmp":
		im=Image.open(file)
		name = os.path.join(path, file)
		print(name)
		print(im.format, im.size, im.mode)
		reim = im.resize((574,480),Image.ANTIALIAS)
		reim.save(name, "JPEG")
