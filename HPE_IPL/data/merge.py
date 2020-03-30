import os
import cv2

def imgs2video(imgs_dir, save_name):
    fps = 24
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(save_name, fourcc, fps, (1920, 1080))
    # no glob, need number-index increasing
    imgs = os.listdir("outImg")
    imgs.sort()

    for imgname in imgs:
        # imgname = os.path.join(imgs_dir, 'core-{:02d}.png'.format(i))
        frame = cv2.imread(os.path.join('outImg', imgname))
        print(imgname)
        video_writer.write(frame)

    video_writer.release()

if __name__ == '__main__':
    imgs_dir = "outImg"
    save_name = "out.avi"
    fps = 24
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(save_name, fourcc, fps, (1920, 1080))
    # no glob, need number-index increasing
    imgs = os.listdir("outImg")
    imgs.sort()

    for imgname in imgs:
        # imgname = os.path.join(imgs_dir, 'core-{:02d}.png'.format(i))
        frame = cv2.imread(os.path.join('outImg', imgname))
        print(imgname)
        video_writer.write(frame)

    video_writer.release()
