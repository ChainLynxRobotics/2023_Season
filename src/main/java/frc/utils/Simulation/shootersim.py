import sympy as sp

v, x, y, theta, g = sp.symbols('v x y theta g')
sol = sp.solve(sp.Eq(v,g*x/(sp.cos(theta)*(v*sp.sin(theta)-sp.sqrt(3*v**2*sp.sin(theta)**2-2*g*y)))), v)
print(sol)

#sqrt(2)*sqrt(-g*x/(sin(theta)*cos(theta)) + g*y/sin(theta)**2 - g*sqrt(3*x**2*sin(theta)**2 - 2*x*y*sin(theta)*cos(theta) + y**2*cos(theta)**2)/(sin(theta)**2*cos(theta)))/2