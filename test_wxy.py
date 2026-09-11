import cv2
d = cv2.imread("/home/rookie/collect/data/20250923_155043/dataset/episode_1/images/1/camera1_depth.png", cv2.IMREAD_UNCHANGED)
print(d.shape, d.dtype, d.min(), d.max())
#yongyujiance shendutu shengcheng shifou zhengque