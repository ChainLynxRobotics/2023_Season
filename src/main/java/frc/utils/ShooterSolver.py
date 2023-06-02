import sympy as sp

v, x, y, theta, g = sp.symbols('v x y theta g')
sol = sp.solve(sp.Eq(v,g*x/(sp.cos(theta)*(v*sp.sin(theta)-sp.sqrt(v**2*sp.sin(theta)**2-2*g*y)))), v)
print(sol)

#[x*sqrt(g/(x*sin(2*theta) - y*cos(2*theta) - y)), -x*sqrt(-g/(-x*sin(2*theta) + y*cos(2*theta) + y))], take positive solution