import numpy as np
import cv2
import glob

def calibrate_cameras():
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    mtx_out = [ [0,0,0],
                [0,0,0],
                [0,0,0] ]
    dist_out = [0,0,0,0,0]

    images = glob.glob('calibration/*.jpg')
    count = 0.

    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            count += 1
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

            for i in range(5):
                dist_out[i] += dist[0][i]

            for i in range(3):
                for j in range(3):
                    mtx_out[i][j] += mtx[i][j]

            cv2.destroyAllWindows()

    for i in range(5):
        dist_out[i] = dist_out[i] / count

    for i in range(3):
        for j in range(3):
            mtx_out[i][j] = mtx_out[i][j] / count

    np.save('dist_param_test', dist_out)
    np.save('mtx_param_test', mtx_out)

    return dist_out, mtx_out

def calibrate_batch():
    dist = np.load('dist_param_test.npy')
    mtx = np.load('mtx_param_test.npy')

    mtx_out = [ [0,0,0],
                [0,0,0],
                [0,0,0] ]
    dist_out = [0,0,0,0,0]

    dist_new, mtx_new = calibrate_cameras()

    for i in range(5):
        dist_out[i] = (dist[i] + dist_new[i]) / 2.

    for i in range(3):
        for j in range(3):
            mtx_out[i][j] = (mtx[i][j] + mtx_new[i][j]) / 2.

    np.save('dist_param_test', dist_out)
    np.save('mtx_param_test', mtx_out)


def test_calibration():
    dist = np.load('dist_param_test.npy')
    mtx = np.load('mtx_param_test.npy')

    img = cv2.imread('calibration226.jpg')

    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # undistort
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    cv2.imshow('dst', dst)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    #calibrate_cameras()
    test_calibration()

    #calibrate_batch()
