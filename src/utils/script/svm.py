from sklearn.metrics import classification_report, confusion_matrix
from sklearn.model_selection import train_test_split

import cv2
import os
import numpy as np

pic_root = './'
classes = ["0", "1", "3", "4", "6", "7"]

data_image = []
data_label = []

for class_ in classes:
    dir_ = os.path.join(pic_root, str(class_))  # 连接地址
    # print(dir_)
    # data_label.extend([class_ for i in range(len(os.listdir(dir_)))])
    for file in os.listdir(dir_):
        image = cv2.imread(os.path.join(dir_, file))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        gray2 = cv2.threshold(
            gray, 0, 255, cv2.THRESH_OTSU | cv2.THRESH_BINARY)

        array_gray = np.array(gray)

        res = array_gray.flatten()  # 折叠为一维数组 二维->一维
        data_image.append(np.array(res.flatten()))  # 一维转0维
        data_label.append(np.array(class_))
data_image = np.array(data_image, dtype=np.float32)
data_label = np.array(data_label, dtype=np.int)


x_train, x_test, y_train, y_test = train_test_split(
    data_image, data_label, test_size=0.2, random_state=3, shuffle=True)  # 使用30%来测试
# x_train = data_image  # 转为使用所有
# y_train = data_label
svm = cv2.ml.SVM_create()
svm.setType(cv2.ml.SVM_C_SVC)
svm.setKernel(cv2.ml.SVM_LINEAR)
svm.setC(10)
svm.setGamma(3)  # 核参数
svm.setDegree(2)  # 仅选择多项式才有用
svm.setTermCriteria((cv2.TermCriteria_EPS, 300, 1e-2))  # 设置迭代结束条件
train_data = cv2.ml.TrainData_create(
    samples=x_train, layout=cv2.ml.ROW_SAMPLE, responses=y_train)
svm.train(train_data)
svm.save("numbers_model.xml")
# svm.train(layout=cv2.ml.ROW_SAMPLE, samples=x_train, responses=y_train)
y_pred = svm.predict(x_test)

# print(y_pred)
# print(len(x_test))
print(classification_report(
    y_test, y_pred[1], target_names=classes))  # 生成文本类分类
print(confusion_matrix(y_test, y_pred[1]))
