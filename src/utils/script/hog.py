import os
import numpy as np
import cv2
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import train_test_split

# 在这里设置参数
winSize = (32, 32)   # 窗口大小
blockSize = (8, 8)  # 块大小
blockStride = (4, 4)  # 块步长
cellSize = (4, 4)  # 在这个区间里面统计角度，cell在block内
nbins = 6  # 180度划分为几个区间

# 定义对象hog，同时输入定义的参数，剩下的默认即可
hog = cv2.HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins)

pic_root = './'
classes = ["0", "1", "2", "3", "4", "6", "7", "8"]

data_image = []
data_label = []

# 获取svm参数


def get_svm_detector(svm):
    sv = svm.getSupportVectors()
    rho, _, _ = svm.getDecisionFunction(0)
    sv = np.transpose(sv)
    return np.append(sv, [[-rho]], 0)


for class_ in classes:
    dir_ = os.path.join(pic_root, str(class_))  # 连接地址
    # print(dir_)
    # data_label.extend([class_ for i in range(len(os.listdir(dir_)))])
    for file in os.listdir(dir_):
        image = cv2.imread(os.path.join(dir_, file))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        hog_feature = hog.compute(gray).reshape((-1,))

        array_gray = np.array(hog_feature)

        res = array_gray.flatten()  # 折叠为一维数组 二维->一维
        data_image.append(np.array(res.flatten()))  # 一维转0维
        data_label.append(np.array(class_))
data_image = np.array(data_image, dtype=np.float32)
data_label = np.array(data_label, dtype=np.int)


# svm训练
x_train, x_test, y_train, y_test = train_test_split(
    data_image, data_label, test_size=0.2, random_state=3, shuffle=True)  # 使用30%来测试

svm = cv2.ml.SVM_create()
svm.setType(cv2.ml.SVM_C_SVC)
svm.setKernel(cv2.ml.SVM_LINEAR)
svm.setC(1)
svm.setGamma(3)  # 核参数
svm.setDegree(2)  # 仅选择多项式才有用
svm.setTermCriteria((cv2.TermCriteria_EPS, 300, 1e-2))  # 设置迭代结束条件
train_data = cv2.ml.TrainData_create(
    samples=x_train, layout=cv2.ml.ROW_SAMPLE, responses=y_train)
svm.train(train_data)
# svm.trainAuto(samples=x_train, layout=cv2.ml.ROW_SAMPLE, responses=y_train)
# grid = cv2.ml.getDefaultGrid(C)
y_pred = svm.predict(x_test)

# print(y_pred)
# print(len(x_test))
print(classification_report(
    y_test, y_pred[1], target_names=classes))  # 生成文本类分类
print(confusion_matrix(y_test, y_pred[1]))

svm.save("numbers_model.xml")


# svm2 = cv2.ml.SVM_create()
# svm2.load("numbers_model.xml")
# hog.setSVMDetector(get_svm_detector(svm))
# hog.save("number_classify.bin")
