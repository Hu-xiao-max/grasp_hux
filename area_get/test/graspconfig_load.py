
class t():
    def __init__(self):
        self.a = 1
    def call(self):
        b=self.a+1
        return b


t=t()
for i in range(10):
    print(t.call())
