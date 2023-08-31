
import numpy as np

class f():
    def __init__(self, x, y):
        self.x = 1
        self.y = 2

    def t(self):
        c=self.x
        c+=1

        return c

ff=f(1,1)
for i in range(3):
   for t in range(10):
       print(t)
       if t>2:
           break
