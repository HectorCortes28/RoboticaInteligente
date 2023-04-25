#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Importar la biblioteca rospy
import rospy

# Importar el tipo de mensaje Point de geometry_msgs.msg
from geometry_msgs.msg import Point

# Inicializar el nodo con el nombre "path_generator"
rospy.init_node('path_generator')

# Crear un publicador que envía mensajes Point en el topic "/set_point"
pub = rospy.Publisher('/set_point', Point, queue_size=10)

# Crear un objeto Point
point = Point()

# Configurar las coordenadas iniciales del punto en (0, 0)
point.x = 0
point.y = 0

# Publicar el punto en el topic "/set_point"
pub.publish(point)

# Esperar 2 segundos
rospy.sleep(2)

# Si se está ejecutando este archivo como un script (no importado como módulo)
if __name__=='__main__':
    try:
        # Mientras el nodo no haya sido cerrado
        while not rospy.is_shutdown():
            # Configurar el siguiente punto en el camino
            point.x = -0.953561
            point.y = -0.918565
            
            # Publicar el punto en el topic "/set_point"
            pub.publish(point)
            
            # Imprimir un mensaje para indicar que se ha alcanzado este punto
            print("point", 1)
            
            # Esperar 4 segundos antes de publicar el siguiente punto
            rospy.sleep(10)

            # Configurar el siguiente punto en el camino
            point.x = 0.873742
            point.y = -0.918565
            
            # Publicar el punto en el topic "/set_point"
            pub.publish(point)
            
            # Imprimir un mensaje para indicar que se ha alcanzado este punto
            print("point", 2)
            
            # Esperar 5 segundos antes de publicar el siguiente punto
            rospy.sleep(10)

            # Configurar el siguiente punto en el camino
            point.x = 0.873742
            point.y = 0.918565
            
            # Publicar el punto en el topic "/set_point"
            pub.publish(point)
            
            # Imprimir un mensaje para indicar que se ha alcanzado este punto
            print("point", 3)
            
            # Esperar 5 segundos antes de publicar el siguiente punto
            rospy.sleep(10)

            # Configurar el siguiente punto en el camino
            point.x = -0.955022
            point.y = 0.918565
            
            # Publicar el punto en el topic "/set_point"
            pub.publish(point)
            
            # Imprimir un mensaje para indicar que se ha alcanzado este punto
            print("point", 4)
            
            # Esperar 5 segundos antes de publicar el siguiente punto
            rospy.sleep(10)

        # Configurar la velocidad lineal y angular en 0 para detener el robot
        point.linear.x = 0
        point.angular.y = 0
        
        # Publicar el punto en el topic "/set_point"
        pub.publish(point)
    
    # Si se ha producido una excepción (por ejemplo, si el usuario ha interrumpido el programa)
    except rospy.ROSInterruptException:
        # Pasar al bloque siguiente
        pass
