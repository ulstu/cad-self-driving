import tensorflow as tf
from keras._tf_keras.keras.callbacks import ModelCheckpoint, Callback
import cv2
import time
import numpy as np
from collections import deque
import math
import os
from keras import datasets, layers, models
from keras._tf_keras.keras.callbacks import ModelCheckpoint
from sklearn.preprocessing import OneHotEncoder
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)


class CNN():
    def __init__(self, load=False, gpu_load=1.0):
        self.load = load
#        self.tf_config = tf.ConfigProto()
#        self.tf_config.gpu_options.allow_growth = True
#        self.tf_config.gpu_options.per_process_gpu_memory_fraction = gpu_load
        self.path_to_model = 'weights_0'
        self.path_to_vis = 'visualize_valid/'
        self.path_to_dataset = '../real_test/'
        self.path_to_vis_new = 'visualize_test/'
        self.epoch = 0
        self.image_channels = 1
        self.test_analize = {}
        self.enc = 0

    def init_model(self, num_classes, labels):
        param = [64,
                 [16, 3, 2],
                 [32, 3, 2],
                 [64, 3, 2],
                 [96, 3, 2],
                 # [128, 5, 2],
                 # [196, 5, 2],
                 # [256, 5, 2],
                 #[512, 5, 2]
                 ]
        
        self.labels = labels
        self.num_classes = num_classes
        self.image_size = param[0]
        is_training = True
        channels = 1
        image_channels = 3
        learning_rate = 0.0001
        self.model = models.Sequential()
        self.model.add(layers.Conv2D(16, (3, 3), activation='relu', input_shape=(param[0], param[0], image_channels)))
        self.model.add(layers.MaxPooling2D((2, 2)))
        self.model.add(layers.Conv2D(32, (3, 3), activation='relu'))
        self.model.add(layers.MaxPooling2D((2, 2)))
        self.model.add(layers.Conv2D(64, (3, 3), activation='relu'))
        self.model.add(layers.Flatten())
        self.model.add(layers.Dense(1024, activation='sigmoid'))
        self.model.add(layers.Dense(num_classes, activation='softmax'))
        self.model.compile(optimizer='adam',
                      #loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
                      loss='categorical_crossentropy',
                      metrics=['accuracy'])
            # self.vars = []
            # activation = tf.nn.relu
            # self.input_X = tf.placeholder(tf.float32, [None, param[0], param[0], image_channels], name='Input_placeholder')
            # self.input_size = param[0]
            # self.encode_layers = [[self.input_X]]
            # c = 0
            # for i in range(1, len(param) - 1):
            #     self.encode_layers.append([])
            #     with tf.variable_scope('CNN/conv_block_/' + str(pow(2, i)) + str(c)):
            #         downa = tc.layers.conv2d(self.encode_layers[i - 1][0], param[i][0], (param[i][1], param[i][1]),
            #                                  normalizer_fn=tc.layers.batch_norm, activation_fn=activation,
            #                                  normalizer_params={'is_training': is_training})
            #
            #         if (param[i][2] == 2):
            #             downc = tc.layers.max_pool2d(downa, (2, 2), padding='same')
            #         else:
            #             downc = downa
            #         self.encode_layers[i].append(downa)
            #         self.encode_layers[i].append(downc)
            #         c += 0
            # flatten = tc.layers.flatten(self.encode_layers[-1][-1])
            # dense = tc.layers.fully_connected(flatten, 1024)
            # self.keep_probability = tf.placeholder(tf.float32)
            # dropout = tc.layers.dropout(dense, self.keep_probability)
            # self.logit_conv = tc.layers.fully_connected(dropout, num_classes, activation_fn=None)
            # # self.y_conv = tf.nn.softmax(self.logit_conv)
            # self.y_conv = tf.nn.sigmoid(self.logit_conv)
            #
            #
            #
            # self.gs = tf.train.get_or_create_global_step()
            # self.input_Y = tf.placeholder(tf.float32, [None, num_classes], name='mask_placeholder')
            # # self.loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=self.logit_conv, labels=self.input_Y))
            # self.loss = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=self.logit_conv, labels=self.input_Y))
            # # a = tf.nn.sig
            # self.lr = tf.train.exponential_decay(learning_rate,
            #                                      global_step=self.gs,
            #                                      decay_rate=0.5,
            #                                      decay_steps=50000)
            # self.optimizer = tc.layers.optimize_loss(loss=self.loss,
            #                                          global_step=self.gs,
            #                                          learning_rate=self.lr,
            #                                          optimizer='Adam')
            # self.argmax_y_conv = tf.argmax(self.y_conv, 1)
            # self.argmax_y_input = tf.argmax(self.input_Y, 1)
            # self.correct_prediction = tf.equal(self.argmax_y_conv, self.argmax_y_input)
            # self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32))
            # self.merged_summary = tf.summary.merge_all()

    def load_model(self, path_to_model_folder):
        path_to_model = os.path.join(path_to_model_folder, "model.h5")
        path_to_labels = os.path.join(path_to_model_folder, "labels.txt")
        
        self.model = tf.keras.models.load_model(path_to_model)
        
        with open(path_to_labels, "r") as file:
            self.labels = [line.strip("\n") for line in file.readlines()]

    def predict(self, images):
        predicted = self.model.predict_on_batch(np.array(images)/255.0)
        return None, predicted


    class SavingCallback(Callback):
        def __init__(self, folder_path, labels):
            self.folder_path = folder_path
            self.labels = labels
        
        def on_epoch_end(self, epoch, logs=None):
            val_accuracy = logs.get('val_accuracy')
            model_path = os.path.join(self.folder_path, f"model(epoch={epoch + 1}, val={round(val_accuracy, 3)}).keras")
            
            for filename in os.listdir(self.folder_path):
                file_path = os.path.join(self.folder_path, filename)
                if os.path.isfile(file_path):
                    if filename.split('.')[-1] == "keras":
                        os.remove(file_path)
            
            self.model.save(model_path)
            
            labels_str = ""
            for idx, label in enumerate(self.labels):
                labels_str += label
                if idx < len(self.labels) - 1:
                    labels_str += '\n'
            
            labels_path = os.path.join(self.folder_path, "labels.txt")
            with open(labels_path, "w") as file:
                file.write(labels_str)


    def train(self, 
              train_dataset, 
              valid_dataset, 
              test_dataset, 
              good_test_dataset,
              path_to_save):
    
        model_path = os.path.join(path_to_save, "model.h5")
        labels_path = os.path.join(path_to_save, "labels.txt")
    
        labels_str = ""
        for idx, label in enumerate(self.labels):
            labels_str += label
            if idx < len(self.labels) - 1:
                labels_str += '\n'
        
        with open(labels_path, "w") as file:
            file.write(labels_str)
        
        checkpoint = ModelCheckpoint(model_path, monitor='val_accuracy', verbose=1, save_best_only=True, mode='max')
        #checkpoint = CNN.SavingCallback(path_to_save, self.labels)
        
        self.model.fit(np.array(train_dataset.data_x)/255.0,
          train_dataset.data_y,
          epochs=50,
          validation_data=(np.array(valid_dataset.data_x)/255.0, valid_dataset.data_y),
          callbacks=[checkpoint])  # Pass callback to training
        
        


    def viz(self, batch_predict_mask, batch_true_mask, batch_image, cc, iter, path):
        for count, images in enumerate(zip(batch_image, batch_true_mask, batch_predict_mask)):
            image, true_mask, predict_mask = images
            if image.shape[2] == 1:
                size = image.shape[0:2]
                image = image.reshape(size)
                image = np.stack((image, image, image), axis=-1)
            # predict_mask = predict_mask[:, :, :3]

            new_predict_mask = np.stack((predict_mask * 255, predict_mask * 255,
                                         predict_mask * 255), axis=-1)
            result = cv2.addWeighted(np.array(image, dtype=np.uint8), 0.5, np.array(new_predict_mask, dtype=np.uint8), 0.5,
                                     0)
            cv2.imwrite(os.path.join(path, '{}_{}_{}.jpg'.format(self.epoch, cc, iter)), result)

    def valid_test(self, dataset, batch_size, is_train = True):
        integral_accuracy = []
        integral_loss = []

        # batches_count = int(len(dataset.data_x) / batch_size)

        if is_train:
            batches_count = 20
        else:
            batches_count = int(len(dataset.data_x) / batch_size)

        for count in range(batches_count):
            batch_xs, batch_y = dataset.next_batch(batch_size)
            loss, accuracy, corrects, y_convs, y_inputs = self.sess.run([self.loss, self.accuracy, self.correct_prediction, self.argmax_y_conv, self.argmax_y_input],
                                         feed_dict={self.input_X: np.array(batch_xs),
                                                    self.input_Y: np.array(batch_y),
                                                    self.keep_probability: 1})

            labels = self.enc.inverse_transform(batch_y)
            for label, correct, y_conv in zip(labels, corrects, y_convs):
                self.test_analize[label[0]][0] += 1
                self.test_analize[label[0]][1] += correct
                if not correct:
                    self.test_analize[label[0]][2][y_conv] += 1
            integral_accuracy += [accuracy]
            integral_loss += [loss]

        accuracy = np.mean(integral_accuracy)
        loss = np.mean(integral_loss)
        return accuracy, loss

    def valid(self, dataset, batch_size, is_train = True):
        integral_accuracy = []
        integral_loss = []

        # batches_count = int(len(dataset.data_x) / batch_size)
        if is_train:
            batches_count = 20
        else:
            batches_count = int(len(dataset.data_x) / batch_size)

        for count in range(batches_count):
            batch_xs, batch_y = dataset.next_batch(batch_size)
            loss, accuracy = self.sess.run([self.loss, self.accuracy],
                                         feed_dict={self.input_X: np.array(batch_xs),
                                                    self.input_Y: np.array(batch_y),
                                                    self.keep_probability: 0.5})
            integral_accuracy += [accuracy]
            integral_loss += [loss]

        accuracy = np.mean(integral_accuracy)
        loss = np.mean(integral_loss)
        return accuracy, loss