import numpy as np, pprint, sys

sys.modules['numpy._core'] = np.core
# p = np.load('/scratch/aadith_warrier/JRDB/bboxes/upper_velodyne/bytes-cafe-2019-02-07_0/000123.npy',
#             allow_pickle=True)
# pprint.pp(p[0])
m = np.load("/scratch/gaurav_kumar/results/masks/upper_velodyne/clark-center-2019-02-28_0/000069.npy",
            allow_pickle=True)
print(np.unique(m, return_counts=True))