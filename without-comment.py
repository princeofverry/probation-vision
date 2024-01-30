# Import library yang diperlukan
from scipy.spatial import distance as dist
from collections import deque
import numpy as np
import argparse
from imutils import perspective
import imutils
import cv2
import math
import urllib
import serial

# Inisialisasi koneksi serial untuk komunikasi dengan perangkat eksternal
ser1 = serial.Serial('/dev/cu.usbmodem1D11401', 9600)

# Fungsi untuk menghitung titik tengah antara dua titik
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

# Parsing argumen dari baris perintah
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

# Inisialisasi lebar objek (dalam satuan yang tidak jelas)
width = 7.87

# Definisi rentang warna untuk deteksi objek
lower = {
    'red': (0, 127, 90),
    'green': (62, 82, 0),
    'blue': (90, 150, 140),
    'yellow': (23, 59, 119),
    'orange': (0, 130, 190),
}
lower['blue'] = (93, 10, 0)

upper = {
    'red': (5, 255, 255),
    'green': (99, 255, 245),
    'blue': (102, 255, 255),
    'yellow': (54, 255, 255),
    'orange': (5, 255, 255)
}

colors = {
    'red': (179, 255, 255),
    'green': (109, 255, 255),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 217),
    'orange': (0, 140, 255)
}

# Inisialisasi kamera
camera = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 ", cv2.CAP_GSTREAMER)

# Nilai tengah (titik tengah gambar)
nilaitengah = str(320)
tolerance_value = 30
pointsList = []

# Loop utama
while True:
    # Pengaturan parameter kamera
    camera.set(28, 255)
    (grabbed, frame) = camera.read()

    # Preprocessing citra
    frame = imutils.resize(frame, width=640, height=960)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Pengolahan warna merah
    kernel1 = np.ones((9, 9), np.uint8)
    mask_red = cv2.inRange(hsv, lower['red'], upper['red'])
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel1)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel1)
    cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Pengolahan warna hijau
    kernel2 = np.ones((9, 9), np.uint8)
    mask_green = cv2.inRange(hsv, lower['green'], upper['green'])
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel2)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel2)
    cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Penggambaran warna dan objek pada frame
    col = ((0, 0, 255), (240, 0, 159), (0, 165, 255), (255, 255, 0), (255, 0, 255), (255, 255, 255))
    refObj_red = None
    refObj_green = None
    box = None
    box_green = None
    box_red = None

    testt = frame.copy()

    # Deteksi bola hijau
    if len(cnts_green) > 0:
        c = max(cnts_green, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Gambar lingkaran dan tulisan pada citra uji
        if radius > 0.5:
            cv2.circle(testt, (int(x), int(y)), int(radius), colors["green"], 2)
            cv2.putText(testt, "green ball", (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors["green"], 2)

            # Inisialisasi objek referensi jika belum ada
            if refObj_green is None:
                refObj_green = (box_green, (x, y), radius)
            else:
                # Cek apakah objek bergerak dalam toleransi jarak tertentu
                if (math.sqrt((center[0] - refObj_green[1][0]) ** 2 + (center[1] - refObj_green[1][1]) ** 2)) < 15:
                    if len(pointsList) > 0:
                        for i in range(len(pointsList)):
                            if (nilaitengah) == pointsList[i]:
                                break
                            else:
                                # Gambar objek di frame
                                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                                cv2.putText(frame, "green ball", (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                x, y, w, h = cv2.boundingRect(c)
                                pointsList.append(str(int(x + w / 2)))

    # Deteksi bola merah
    if len(cnts_red) > 0:
        c = max(cnts_red, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Gambar lingkaran dan tulisan pada citra uji
        if radius > 0.5:
            cv2.circle(testt, (int(x), int(y)), int(radius), colors["red"], 2)
            cv2.putText(testt, "red ball", (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors["red"], 2)

            # Inisialisasi objek referensi jika belum ada
            if refObj_red is None:
                refObj_red = (box_red, (x, y), radius)
            else:
                # Cek apakah objek bergerak dalam toleransi jarak tertentu
                if (math.sqrt((center[0] - refObj_red[1][0]) ** 2 + (center[1] - refObj_red[1][1]) ** 2)) < 15:
                    if len(pointsList) > 0:
                        for i in range(len(pointsList)):
                            if (nilaitengah) == pointsList[i]:
                                break
                            else:
                                # Gambar objek di frame
                                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
                                cv2.putText(frame, "red ball", (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                x, y, w, h = cv2.boundingRect(c)
                                pointsList.append(str(int(x + w / 2)))

    # Tambahkan teks pada frame
    cv2.putText(frame, "COB", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Tampilkan frame dan citra uji
    cv2.imshow("Frame", frame)
    cv2.imshow("Test", testt)

    # Tunggu input keyboard
    key = cv2.waitKey(1) & 0xFF

    # Keluar dari loop jika tombol 'q' ditekan
    if key == ord("q"):
        break
    # Reset pointsList jika tombol 'r' ditekan
    elif key == ord("r"):
        pointsList = []

    # Proses kontrol berdasarkan posisi titik terakhir dalam pointsList
    if len(pointsList) > 0:
        if int(nilaitengah) > (int(pointsList[len(pointsList) - 1]) + tolerance_value):
            ser1.write(b'l')
            print("kiri")
        elif int(nilaitengah) < (int(pointsList[len(pointsList) - 1]) - tolerance_value):
            ser1.write(b'r')
            print("kanan")
        elif (int(pointsList[len(pointsList) - 1]) - tolerance_value) <= int(nilaitengah) <= (
                int(pointsList[len(pointsList) - 1]) + tolerance_value):
            ser1.write(b's')
            print("lurus")
        elif int(nilaitengah) == (int(pointsList[len(pointsList) - 1])):
            ser1.write(b's')
            print("lurus")
        else:
            ser1.write(b's')
            print("lurus")
