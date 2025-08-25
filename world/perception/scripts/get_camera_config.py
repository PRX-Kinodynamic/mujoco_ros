import os
import re
import argparse

# TODO: Would be nice to convert this into a service where another process requests the device by camera id.
def get_list_of_devices():
	# devices = subprocess.run(["v4l2-ctl", "--list-devices"]) 
	devices = os.popen("v4l2-ctl --list-devices").read()
	m = re.findall(r'/dev/video\d+', devices)
	return m

def find_from_id(id, devices):
	for dev in devices:
		dev_info = os.popen(f"v4l2-ctl --device={dev} --all").read()
		# print(f"{dev} - {dev_info}")
		m = re.search(f"{id}", dev_info)
		if m is not None:
			# print(dev_info)
			# print(f"{dev} - {m.group(0)}")
	# Pixel Format      : 'YUYV' (YUYV 4:2:2)
			m_YUYV = re.search(f"YUYV", dev_info)
			m_MJPG = re.search(f"MJPG", dev_info)
			if m_YUYV is not None:
				print(f"FORMAT: {dev} - {m_YUYV.group(0)}")
			if m_MJPG is not None :
				print(f"FORMAT: {dev} - {m_MJPG.group(0)}")



if __name__ == '__main__':
	parser = argparse.ArgumentParser(prog="get_camera_config", description="Get the device /dev/videoN by camera ID")
	parser.add_argument('-i', '--id', help="Id of camera", required=True)

	args = parser.parse_args();

	devices = get_list_of_devices()
	find_from_id(args.id, devices)
