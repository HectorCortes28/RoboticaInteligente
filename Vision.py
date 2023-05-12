import cv2 as cv
import numpy as np


 

def detectar_circulos(frame, prevCircle):
    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurFrame = cv.GaussianBlur(grayFrame, (13, 13), 0)
    
    # Detección de círculos en la imagen suavizada
    # Los parámetros de HoughCircles son los siguientes: imagen, método de detección, inverso de la resolución, distancia mínima entre los centros de los círculos, param1, param2, minRadius, maxRadius
    # Param1: umbral para el detector de bordes interno. Mientras más alto, menos círculos se detectan
    # Param2: umbral para el detector de centros. Mientras más bajo, menos círculos se detectan
    # MinRadius: radio mínimo del círculo a detectar
    # MaxRadius: radio máximo del círculo a detectar
    circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1.2 , 100, param1=50, param2=120, minRadius=10, maxRadius=300)


    # Elegir el círculo más cercano al anteriorq
    if circles is not None:
        # Convertir los parámetros de los círculos (x, y, radio) a enteros
        circles = np.uint16(np.around(circles))
        choosen = None
 
        for i in circles[0, :]:
            if choosen is None: choosen = i
            if prevCircle is not None:
                if dist(choosen[0], choosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0], i[1], prevCircle[0], prevCircle[1]):
                    choosen = i

        return choosen
 
    return None

 

def detectar_color(frame, choosen):

    # Convertir la imagen a espacio de color HSV
    hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

 

    # Definir los rangos de los colores a detectar
    colors = {"red": ([0, 70, 50], [5, 255, 255]),
              "yellow": ([20, 100, 100], [30, 255, 255]),
              "green": ([40, 50, 50], [80, 255, 255])}

   

    # Buscar el color con mayor porcentaje de píxeles en la máscara
    max_percent = 0
    color_name = "unknown"
    for name, (lower, upper) in colors.items():
        mask = cv.inRange(hsvFrame, np.array(lower), np.array(upper))
        percent = cv.countNonZero(mask) / (mask.shape[0] * mask.shape[1])
        if percent > max_percent:
            max_percent = percent
            color_name = name


    if color_name != "unknown":
        cv.putText(frame, color_name.capitalize(), (choosen[0], choosen[1]), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)
    else:
        print("No detecto color")



# Dibujar los círculos detectados
def dibujar_circulos(frame, choosen):
    cv.circle(frame, (choosen[0], choosen[1]), 1, (0, 255, 0), 3)
    cv.circle(frame, (choosen[0], choosen[1]), choosen[2], (255, 0, 255), 3)

 

# Calcular la distancia entre dos puntos
def dist(x1, y1, x2, y2):
    return (float(x1) - float(x2))**2 + (float(y1) - float(y2))**2

 
def main():

    videoCapture = cv.VideoCapture(0) 
    prevCircle = None
 
    while True:
        ret, frame = videoCapture.read()

        if not ret: break

        choosen = detectar_circulos(frame, prevCircle)
        if choosen is not None:
            detectar_color(frame, choosen)
            dibujar_circulos(frame, choosen)
            prevCircle = choosen
 

        cv.imshow('Circles', frame)

        if cv.waitKey(1) == ord('q'): break

 

    videoCapture.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()
