import torch
import torch.nn as nn
import torch.optim as optim

# 生成训练数据
x = torch.randn(100, 1) * 10
y = 2 * x + 1 + torch.randn(100, 1)

# 定义线性模型
class LinearModel(nn.Module):
    def __init__(self):
        super(LinearModel, self).__init__()
        self.linear = nn.Linear(1, 1)

    def forward(self, x):
        return self.linear(x)
    

def train():
    # 训练模型
    model = LinearModel()
    criterion = nn.MSELoss()
    optimizer = optim.SGD(model.parameters(), lr=0.001)

    for epoch in range(100):
        optimizer.zero_grad()
        outputs = model(x)
        loss = criterion(outputs, y)
        loss.backward()
        optimizer.step()

        if (epoch + 1) % 10 == 0:
            print(f'Epoch: {epoch + 1}, Loss: {loss.item()}')

    print("模型权重：", model.linear.weight.item())
    print("模型偏置：", model.linear.bias.item())

    # 保存模型
    torch.save(model.state_dict(), './net/fine_tune/linear_model.pth')

def fine_tune():
    t = torch.randn(1000, 1) * 10
    z = 2 * t + 1 + torch.randn(1000, 1)
    # 加载模型
    loaded_model = LinearModel()
    loaded_model.load_state_dict(torch.load('./net/fine_tune/linear_model.pth'))
    loaded_model.eval()
    criterion = nn.MSELoss()

    # Fine-tune模型
    fine_tune_optimizer = optim.SGD(loaded_model.parameters(), lr=0.0001)

    for epoch in range(100):
        fine_tune_optimizer.zero_grad()
        outputs = loaded_model(t)
        loss = criterion(outputs, z)
        loss.backward()
        fine_tune_optimizer.step()

        if (epoch + 1) % 10 == 0:
            print(f'Fine-tune Epoch: {epoch + 1}, Loss: {loss.item()}')

    # 保存模型
    torch.save(loaded_model.state_dict(), './net/fine_tune/linear_model_fine_tune.pth')

    # 输出模型参数
    print("fine tune模型权重：", loaded_model.linear.weight.item())
    print("fine tune模型偏置：", loaded_model.linear.bias.item())

def fine_tune_change_net():
    # 加载模型
    loaded_model = LinearModel()
    loaded_model.load_state_dict(torch.load('./net/fine_tune/linear_model.pth'))
    loaded_model.eval()
    criterion = nn.MSELoss()
    # 在原始模型基础上添加一个全连接层
    extended_model = nn.Sequential(
        loaded_model,
        nn.Linear(1, 1)
    )

    # Fine-tune模型
    fine_tune_optimizer = optim.SGD(extended_model.parameters(), lr=0.0001)

    for epoch in range(100):
        fine_tune_optimizer.zero_grad()
        outputs = extended_model(x)
        loss = criterion(outputs, y)
        loss.backward()
        fine_tune_optimizer.step()

        if (epoch + 1) % 10 == 0:
            print(f'Fine-tune Epoch: {epoch + 1}, Loss: {loss.item()}')

    t=torch.randn(1, 1)
    print(extended_model.forward(t))

    # 输出模型参数
    print("原始模型权重：", extended_model[0].linear.weight.item())
    print("原始模型偏置：", extended_model[0].linear.bias.item())
    print("全连接层权重：", extended_model[1].weight.item())
    print("全连接层偏置：", extended_model[1].bias.item())




def test():
    loaded_model = LinearModel()
    loaded_model.load_state_dict(torch.load('./net/fine_tune/linear_model_fine_tune.pth'))
    loaded_model.eval()
    # 输出模型参数
    t=torch.randn(1, 1)
    print(loaded_model.forward(t))
    print("fine tune模型权重：", loaded_model.linear.weight.item())
    print("fine tune模型偏置：", loaded_model.linear.bias.item())


if __name__=='__main__':
    # train()
    # fine_tune()
    # test()
    fine_tune_change_net()

