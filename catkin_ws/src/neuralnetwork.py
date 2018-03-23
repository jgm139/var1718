import numpy as np
import csv

from keras.preprocessing.image import load_img
from keras.models import Sequential
from keras.layers import Dense, Flatten, MaxPooling2D
from keras.layers.convolutional import Conv2D
from keras.callbacks import EarlyStopping
from keras import backend as K

batch_size = 2
epochs = 2
img_rows, img_cols = 224, 224

# Set image channels order
K.set_image_data_format('channels_last')

def load_data():
    image_list = []
    Y = []
    with open('datosNav.csv') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=';')
        for row in spamreader:
            im = load_img(row[0]+".png", grayscale=True, target_size=[img_rows, img_cols])  # this is a PIL image
            image_list.append(np.asarray(im).astype('float32')/255)
            Y.append([float(row[1]), float(row[2])])

    n = len(image_list)    # Total examples
    X = np.asarray(image_list).reshape(n, img_rows, img_cols, 1)
    Y = np.asarray(Y, "float32")
    input_shape = (img_rows, img_cols, 1)

    msk = np.random.rand(len(X)) < 0.9 # Train 90% and Test 10%
    X_train, X_test = X[msk], X[~msk]
    Y_train, Y_test = Y[msk], Y[~msk]

    return X_train, X_test, Y_train, Y_test, input_shape


def cnn_model(input_shape):
    model = Sequential()
    model.add(Conv2D(32, kernel_size=(5, 5), strides=(1, 1),
                    activation='relu',
                    input_shape=input_shape))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
    model.add(Conv2D(64, (5, 5), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Flatten())
    model.add(Dense(1000, activation='relu'))
    model.add(Dense(2, activation='tanh'))

    return model


##################################################################################
# Main program

# the data split between train and test sets
np.set_printoptions(threshold=np.nan)
X_train, X_test, Y_train, Y_test, input_shape = load_data()

print(X_train.shape, 'train samples')
print(X_test.shape, 'test samples')
print(Y_train.shape, 'train samples')
print(Y_test.shape, 'test samples')
print(img_rows, 'x', img_cols, 'image size')
print(input_shape, 'input_shape')
print(epochs, 'epochs')

model = cnn_model(input_shape)
print(model.summary())

model.compile(loss='categorical_crossentropy', optimizer='rmsprop', metrics=['accuracy'])
early_stopping = EarlyStopping(monitor='loss', patience=3)

model.fit(X_train, Y_train, batch_size=batch_size, epochs=epochs, verbose=2, validation_data=(X_test, Y_test), callbacks=[early_stopping])

#
# Results
#
#loss, acc = model.evaluate(X_test, Y_test, verbose=0)
#print('Test score:{:.2f} accuracy: {:.2f}%'.format(loss, acc*100))

#print (model.predict(X_train))
