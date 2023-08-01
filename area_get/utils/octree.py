import numpy as np

class OctreeNode:
    def __init__(self, center, size):
        self.center = center
        self.size = size
        self.children = [None] * 8
        self.points = []

    def insert(self, point):
        '''
        向节点中插入一个点。如果当前节点没有子节点且存储的点云数量小于 1
        （这里只允许存储一个点，可以根据需要调整），则将点添加到 points 列表中。
        否则，将点插入到对应的子节点中。如果当前节点还没有子节点，
        需要先进行细分（调用 subdivide 方法）。
        '''
        if len(self.points) < 1 and all(child is None for child in self.children):
            self.points.append(point)
            return

        if all(child is None for child in self.children):
            self.subdivide()

        for child in self.children:
            if child.contains(point):
                child.insert(point)
                return

    def subdivide(self):
        for i in range(8):
            offset = np.array([(i >> 0) & 1, (i >> 1) & 1, (i >> 2) & 1]) * 0.5 - 0.25
            child_center = self.center + offset * self.size
            child_size = self.size * 0.5
            self.children[i] = OctreeNode(child_center, child_size)

        for point in self.points:
            self.insert(point)

        self.points.clear()

    def contains(self, point):
        return all(abs(point - self.center) <= self.size * 0.5)

    def intersect(self, other):
        return all(abs(self.center - other.center) <= (self.size + other.size) * 0.5)

    def query(self, node):
        if self.intersect(node):
            if any(child is not None for child in self.children):
                return any(child.query(node) for child in self.children)
            else:
                return any(node.contains(point) for point in self.points)
        else:
            return False

def collision_detection(points1, points2, size):
    '''
    这个函数接收两组点云数据（points1 和 points2）以及八叉树的初始尺寸
    （size）,然后创建两个八叉树根节点（root1 和 root2）,分别插入点云数据。
    最后调用 root1 的 query 方法检测与 root2 是否存在碰撞，并返回结果。
    '''

    root1 = OctreeNode(np.array([0, 0, 0]), size)
    root2 = OctreeNode(np.array([0, 0, 0]), size)

    for point in points1:
        root1.insert(point)

    for point in points2:
        root2.insert(point)

    return root1.query(root2)

if __name__ == "__main__":
    points1 = np.array([[0, 0, 0], [0.1, 0.1, 0.1], [0.2, 0.2, 0.2]])
    points2 = np.array([[0.25, 0.25, 0.25], [0.35, 0.35, 0.35], [0.45, 0.45, 0.45]])
    size = 1#size：节点的尺寸，表示该节点所表示的立方体区域的边长

    result = collision_detection(points1, points2, size)
    print("Collision detected" if result else "No collision detected")