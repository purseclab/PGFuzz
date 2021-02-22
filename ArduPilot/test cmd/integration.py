import numpy as np
from scipy.integrate import simps, quad
import scipy.integrate as integrate

def r(s):
	return s

def x(s):
	return s

def integrand(s, r, x, w):
	#return a*x**2 + b
	return abs(r - x)/w
t = 1
w = 10
a = 2
b = 1

d1 = np.array([1, 2, 3, 4, 5])
d2 = np.array([2, 3, 4, 5, 6])

I = quad(integrand, t, t+w, args=(r(d1), x(d2), w))

print(I)
