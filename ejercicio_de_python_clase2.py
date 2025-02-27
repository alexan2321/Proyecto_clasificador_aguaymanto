


# Algoritmo que te permite ingresar 5 numeros y luego ordeanalos e menor a mayor 


numeros = [int(input("Ingrese un número: ")) for _ in range(5)]

numeros.sort()


print("Números ordenados:", numeros)






# Algoritmo de multiplicacion de dos listas con numpy

import numpy as np


resultado = np.array([float(input(f"Ingrese número {i+1}: ")) for i in range(5)]) * \
           np.array([float(input(f"Ingrese número {i+1}: ")) for i in range(5)])

print("Resultado:", resultado)
