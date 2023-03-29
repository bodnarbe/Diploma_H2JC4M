from math import sqrt

D = 4

try:
    D = sqrt(-4)
    print("Mukodik")
except ValueError:
    print(D)
    print("Nem lehet gyokot vonni")