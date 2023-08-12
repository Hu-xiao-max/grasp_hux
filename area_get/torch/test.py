import torch
import torch.nn as nn
import torch.nn.functional as F


class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Sequential(
            nn.Conv2d(1,1,kernel_size=5, padding=1,stride=2),
            nn.MaxPool2d(kernel_size=3, padding=0,stride=2),
            nn.Conv2d(1,1,kernel_size=3, padding=1,stride=1),

        )

    def forward(self,x):
        x = self.conv1(x)
        return x
    
cnn=CNN()
input=torch.randn(1,200,200)
output=cnn.forward(input)
print(output.shape)
      
#输出大小 = (输入大小 + 2 * padding - kernel size) / stride + 1
'''
所以上述代码完成的操作是
输入200*200
conv1 -> (200-5+2)/2+1 = 99
maxpool -> (99-3)/2+1=49
con2 -> (49-3+2)/1+1 = 49
'''