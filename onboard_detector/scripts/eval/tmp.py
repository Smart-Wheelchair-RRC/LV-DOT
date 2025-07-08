import numpy as np, pprint, sys

sys.modules['numpy._core'] = np.core
p = np.load('/scratch/aadith_warrier/JRDB/bboxes/upper_velodyne/bytes-cafe-2019-02-07_0/000123.npy',
            allow_pickle=True)
pprint.pp(p[0])
