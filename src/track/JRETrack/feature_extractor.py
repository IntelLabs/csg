
import numpy as np 
import cv2
import tensorflow as tf 

def _run_in_batches(func, data_dict, out, batch_size):
    data_len = len(out)
    num_batches = int(data_len / batch_size)

    s, e = 0, 0
    for i in range(num_batches):
        s, e = i * batch_size, (i + 1) * batch_size
        batch_data_dict = {k: v[s:e] for k, v in data_dict.items()}
        out[s:e] = func(batch_data_dict)
    if e < len(out):
        batch_data_dict = {k: v[e:] for k, v in data_dict.items()}
        out[e:] = func(batch_data_dict)


class FeatureExtractor(object): 
    def __init__(self, ckpt): 
        self.sess = tf.Session()
        with tf.gfile.GFile(ckpt, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
        
        self.sess.graph.as_default()    
        tf.import_graph_def(graph_def, name="")
        self.input_var = tf.get_default_graph().get_tensor_by_name("input:0" )
        self.output_var = tf.get_default_graph().get_tensor_by_name("head/out_emb:0")

        self.feature_dim = self.output_var.get_shape().as_list()[-1]
        self.image_shape = self.input_var.get_shape().as_list()[1:]#(h,w,3)
        self.aspect_ratio = self.image_shape[0]/self.image_shape[1]

    def extract(self, image, tlbrs,batch_size=32): 
        image_patches =[]
        for tlbr in tlbrs: 
            #crop image patch 
            xywh = tlbr.copy()
            xywh[2:] = tlbr[2:]- tlbr[:2]
            patch = self.crop_resize_image(image, xywh)
            image_patches.append(patch)
        image_patches = np.asarray(image_patches)
        out = np.zeros((len(image_patches), self.feature_dim), np.float32)
        _run_in_batches(
            lambda x: self.sess.run(self.output_var, feed_dict=x),
            {self.input_var: image_patches}, out, batch_size)  
        return out

    def crop_resize_image(self, image, xywh): 
        tlbr = xywh.copy()
        new_width = self.aspect_ratio * xywh[3]
        tlbr[0] -= (new_width - xywh[2]) / 2
        tlbr[2] = new_width
        tlbr[2:] += tlbr[:2]
        tlbr = tlbr.astype(np.int)
        tlbr[:2] = np.maximum(0, tlbr[:2])
        tlbr[2:] = np.minimum(np.asarray(image.shape[:2][::-1]) - 1, tlbr[2:])
        sx, sy, ex, ey = tlbr
        cropped = image[sy:ey, sx:ex].copy()
        cropped = cv2.resize(cropped, (self.image_shape[1], self.image_shape[0]))
        return cropped



