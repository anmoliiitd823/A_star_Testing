from random import randint
import math
f = open('matrix.txt','w') 
maxi = 5
a = []
for i in range(500):
	b = []
	for j in range(500):
		x = randint(0,1000)
		if(x==10):
			b.append(maxi)
		else:
			b.append(0)
	a.append(b)

w, h = 500, 500;
Matrix = [[0 for x in range(w)] for y in range(h)] 
for i in range(500):
	for j in range(500):
		current = a[i][j]
		if(current==5):
			for x in range(500):	
				for y in range(500):
					if((x!=i or y!=j) and current//(math.sqrt((x-i)**2+(y-j)**2))!=0 and a[x][y]<maxi):
						Matrix[x][y]=int(Matrix[x][y]+(current//(math.sqrt((x-i)**2+(y-j)**2))))
					


for i in range(500):
	for j in range(500):
		a[i][j]=a[i][j]+Matrix[i][j]
		if(a[i][j]>maxi):
			a[i][j]=maxi
		f.write(str(a[i][j])+'\n')

print(f) 
f.close()
