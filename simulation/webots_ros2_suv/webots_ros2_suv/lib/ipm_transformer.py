import numpy as np
import cv2

class IPMTransformer(object):
    def __init__(self, homography_matrix=None) -> None:
        self.__h = homography_matrix

    def calc_homography(self, pts_src, pts_dst):
        self.__h, status = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC)
        return self.__h

    def get_homography_matrix(self):
        return self.__h

    def get_ipm(self, im_src, dst_size=(1200, 800), horizont=0, is_mono=False, need_cut=True):
        im_src = im_src[horizont:, :]
        im_dst = cv2.warpPerspective(im_src, self.__h, dst_size)
        if not need_cut:
            return im_dst
        if not is_mono:
            rows = np.sum(im_dst, axis=(1, 2))
            cols = np.sum(im_dst, axis=(0, 2))
        else:
            rows = np.sum(im_dst, axis=(1))
            cols = np.sum(im_dst, axis=(0))

        nonzero_rows = len(rows[np.nonzero(rows)])
        nonzero_cols = len(cols[np.nonzero(cols)])
        im_dst = im_dst[:nonzero_rows, :nonzero_cols]
        im_dst = cv2.resize(im_dst, (im_dst.shape[1], im_dst.shape[1]), interpolation=cv2.INTER_AREA)
        return im_dst

