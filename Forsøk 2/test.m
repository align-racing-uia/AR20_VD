%deg fra 2 vektorer 

FOYV= [1 0 1]
FNYV = [0 0 0]

a=[0,1]
b=[FOYV(1)-FNYV(1) , FOYV(3)-FNYV(3)]
alpha = acos((a(1) * b(1) + a(2) * b(2)) / (sqrt((a(1)^2 + a(2)^2)) * sqrt((b(1)^2 + b(2)^2))))
alpha = rad2deg(alpha)