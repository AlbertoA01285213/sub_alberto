import numpy as np
import cv2 as cv
import glob
import os

'''
https://www.youtube.com/watch?v=yKypaVl6qQo
'''

chessboardSize = (9, 6)
frameSize = (1920, 1080)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

objp = objp * 20 # Creo que es el tamano de los cuadrados en mm

flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE + cv.CALIB_CB_FAST_CHECK

objpoints = []
imgpointsL = []
imgpointsR = []

home = os.path.expanduser('~')
path_left = os.path.join(home, 'Desktop/Fotos_sub/camara_izq/*.jpg')
path_right = os.path.join(home, 'Desktop/Fotos_sub/camara_der/*.jpg')

imagesLeft = sorted(glob.glob(path_left))
imagesRight = sorted(glob.glob(path_right))

print(f"🔍 Imágenes encontradas: Izquierda: {len(imagesLeft)}, Derecha: {len(imagesRight)}")

def enhance_chessboard(gray_img):
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    return clahe.apply(gray_img)

if len(imagesLeft) == 0:
    print("❌ Error: No se encontraron archivos .jpg en las rutas especificadas.")
    exit()

for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    imgL = cv.imread(imgLeft)
    imgR = cv.imread(imgRight)
    # grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    # grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    grayL = imgL[:,:,1]
    grayR = imgR[:,:,1]

    grayL = enhance_chessboard(grayL)
    grayR = enhance_chessboard(grayR)

    # cv.imshow('Debugging', grayL)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    # retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
    # retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)

    retL, cornersL = cv.findChessboardCornersSB(grayL, chessboardSize, flags)
    retR, cornersR = cv.findChessboardCornersSB(grayR, chessboardSize, flags)

    if retL and retR:

        objpoints.append(objp)

        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)

        print(f"✅ Tablero detectado en: {os.path.basename(imgLeft)}")

    else:
        print(f"⚠️ No se detectó tablero en el par: {os.path.basename(imgLeft)}")

if len(objpoints) == 0:
    print("❌ Error: No se detectó el tablero en NINGUNA imagen. Revisa la iluminación o el 'chessboardSize'.")
    exit()

# Calibracion ==================================
print("⏳ Calibrando cámaras individuales...")
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))

# Stereo vision calibration ===============================

flags = 0
flags |= cv.CALIB_FIX_INTRINSIC # Este valor puede cambiar

criteria_stereo = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL ...)

retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR, 
    newCameraMatrixL, distL, 
    newCameraMatrixR, distR, 
    frameSize, criteria=criteria_stereo, flags=flags
)
# Stereo rectification ===================================

# alpha=0 recorta la imagen para eliminar bordes negros; alpha=1 mantiene todos los píxeles
rectifyScale = 0
# rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv.stereoRectify(newCameraMatrixL, distL, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale, (0,0))

rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv.stereoRectify(
    newCameraMatrixL, distL, 
    newCameraMatrixR, distR, 
    frameSize, rot, trans, 
    alpha=rectifyScale
)

# stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
# stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, frameSize, cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, frameSize, cv.CV_16SC2)

cv_file = cv.FileStorage('stereoMap.xml', cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x', stereoMapL[0])
cv_file.write('stereoMapL_y', stereoMapL[1])
cv_file.write('stereoMapR_x', stereoMapR[0])
cv_file.write('stereoMapR_y', stereoMapR[1])
cv_file.write('q_matrix', Q)

cv_file.release()