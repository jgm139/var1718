import numpy as np
from keras.applications.mobilenet import MobileNet
from keras.preprocessing.image import load_img
from keras.models import Model
from keras.layers import Dense, Input
from keras import backend as K

batch_size = 2
epochs = 2
img_rows, img_cols = 224, 224
input_tensor = Input(shape=(img_rows,img_cols,3))
input_shape = (img_rows, img_cols, 3)
ImageList = ["RobotImages/image411.png", "RobotImages/image338.png", "RobotImages/image90.png", "RobotImages/image220.png", "RobotImages/image431.png"]

# Set image channels order
K.set_image_data_format('channels_last')

def cnn_model(input_shape):
    mobilenet_model = MobileNet(input_shape=(img_rows, img_cols, 3),
                                input_tensor=input_tensor,
                                pooling='avg',
                                include_top=False,
                                weights=None)
    x = mobilenet_model.output
    prediction = Dense(1, activation="sigmoid")(x)
    
    model = Model(inputs=input_tensor, outputs=prediction)

    return model


##################################################################################
# Main program

model = cnn_model(input_shape)

model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
model.load_weights('./IRW.hdf5')

ImageList_ = []
for item in ImageList:
  im = load_img("./"+item, target_size=[img_rows, img_cols])  # this is a PIL image
  ImageList_.append(np.asarray(im).astype('float32')/255)
ImageList_ = np.asarray(ImageList_).reshape(5, img_rows, img_cols, 3)

print (model.predict(ImageList_))
