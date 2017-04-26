material a drf=0 srf=0 rff=1 c1=1.000287566 c2=1.3412e-18 c3=3.777e-32
material r drf=1.0 srf=0.0 rff=0.0

circle r/a (32) 0 0 0 20
box a/r -5 5 5 -5 named target
ray i=1.0 l=5e-7 t=10 | -10 10 20 10
