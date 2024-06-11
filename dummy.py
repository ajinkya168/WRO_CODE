import time
while True:
	with open("Data.txt","w") as f:
		f.write("1")
		time.sleep(3)
		f.write("2")
