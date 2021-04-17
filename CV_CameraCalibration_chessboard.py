import numpy as np
import cv2 as cv
import glob

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

FACTOR = 2
factor = str(FACTOR)
if FACTOR == 0:
    scale_factor = 3.5 / 3                  # 1.1667
if FACTOR == 1:
    scale_factor = ((9 + (5.75 / 16)) / 8)  # 1.1699
if FACTOR == 2:
    scale_factor = (149 / 5) / 25.4         # 1.1732
print("scale_factor: ", scale_factor)

##print("scale_factor_mm: ", scale_factor_mm)
objp = objp * scale_factor

##print("objp: ", objp)

objpoints = []
imgpoints = []

images = glob.glob('*.png')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (3,3), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, (9,6), corners2, ret)
##        cv.imshow('img', img)
##        cv.waitKey(0)

cv.destroyAllWindows()

ret, mtx, dst, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints,
                                                 gray.shape[::-1], None, None)

print("ret: ", ret)
print("mtx: ", mtx)                                                                                                                                      
print("dst: ", dst)

np.savez("CV_ChessboardCalibrationMatrices_scale"+factor+".npz", k=mtx, dist=dst)

##images2 = glob.glob('.png')
##
##for fname in images2:
##    h, w = img.shape[:2]
##    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dst, (w,h), 1, (w,h))
##
##    undistort = cv.undistort(img, mtx, dst, None, newcameramtx)
##
##    x, y, w, h = roi
##    undistort = undistort[y:y+h, x:x+w]
##    cv.imshow("undistorted", undistort)
##    cv.waitKey(0)
##cv.destroyAllWindows()
