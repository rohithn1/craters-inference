import cv2, numpy as np, time, natsort
from pathlib import Path
from rosbags.highlevel import AnyReader

BAG_DIR = Path('/Users/user/craters-inference/data/rosbags')
BAG_PREFIX = 'recording_20251012_151148'
TOPIC_IMAGE = '/csi_cam_0/image_raw'
TOPIC_IMU   = '/imu'

bag_files = natsort.natsorted(BAG_DIR.glob(f'{BAG_PREFIX}_*.bag'))

for bag in bag_files:
    print(f'Playing {bag.name}')
    with AnyReader([bag]) as reader:
        img_conns = [c for c in reader.connections if c.topic == TOPIC_IMAGE]
        imu_conns = [c for c in reader.connections if c.topic == TOPIC_IMU]
        imu_iter  = reader.messages(connections=imu_conns)
        img_iter  = reader.messages(connections=img_conns)
        imu_data  = {k:0 for k in ['ax','ay','az','gx','gy','gz']}

        for (ci,ti,ri),(cm,tm,rm) in zip(img_iter,imu_iter):
            im = reader.deserialize(ri, ci.msgtype)
            imu = reader.deserialize(rm, cm.msgtype)

            imu_data.update({
                'ax': imu.linear_acceleration.x,
                'ay': imu.linear_acceleration.y,
                'az': imu.linear_acceleration.z,
                'gx': imu.angular_velocity.x,
                'gy': imu.angular_velocity.y,
                'gz': imu.angular_velocity.z,
            })

            img = np.frombuffer(im.data, np.uint8).reshape((im.height, im.width, -1)).copy()
            img = cv2.rotate(img, cv2.ROTATE_180)
            overlay = img.copy()
            cv2.rectangle(overlay,(10,10),(350,150),(0,0,0),-1)
            cv2.addWeighted(overlay,0.4,img,0.6,0,img)

            y=40
            for k,v in imu_data.items():
                cv2.putText(img,f'{k}: {v:+.2f}',(20,y),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
                y+=20

            cv2.putText(img,f'{bag.name}',(20,im.height-20),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)
            cv2.imshow('Camera + IMU Overlay',img)
            if cv2.waitKey(33)==27:  # ESC to quit
                cv2.destroyAllWindows()
                exit()
cv2.destroyAllWindows()
