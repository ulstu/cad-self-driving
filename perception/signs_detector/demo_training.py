from dataset_cnn import *
from cnn import *


dataset = DatasetTrain(path_data="signs_data_classify/")
dataset.load()

model = CNN()
model.init_model(num_classes=dataset.get_num_classes(), labels=dataset.list_signs)
model.train(dataset, dataset, dataset, dataset, path_to_save="weights/model-ep50-signs16/")