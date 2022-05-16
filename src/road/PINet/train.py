#############################################################################################################
##
##  Source code for training. In this source code, there are initialize part, training part, ...
##
#############################################################################################################

import cv2
import torch
import visdom
#import sys
#sys.path.append('/home/kym/research/autonomous_car_vision/lanedection/code/')
import agent
import numpy as np
from data_loader import Generator
from parameters import Parameters
import test
import evaluation
import util
import os
import copy

p = Parameters()

###############################################################
##
## Training
## 
###############################################################
def Training():
    print('Training')

    ####################################################################
    ## Hyper parameter
    ####################################################################
    print('Initializing hyper parameter')

    vis = visdom.Visdom()
    loss_window = vis.line(X=torch.zeros((1,)).cpu(),
                           Y=torch.zeros((1)).cpu(),
                           opts=dict(xlabel='epoch',
                                     ylabel='Loss',
                                     title='Training Loss',
                                     legend=['Loss']))
    
    #########################################################################
    ## Get dataset
    #########################################################################
    print("Get dataset")
    loader = Generator()

    ##############################
    ## Get agent and model
    ##############################
    print('Get agent')

    if p.model_path == "":
        lane_agent = agent.Agent()
    else:
        lane_agent = agent.Agent()
        lane_agent.load_weights(0, "tensor(1.3984)")

    ##############################
    ## Check GPU
    ##############################
    print('Setup GPU mode')
    if torch.cuda.is_available():
        lane_agent.cuda()
        #torch.backends.cudnn.benchmark=True

    ##############################
    ## Loop for training
    ##############################
    print('Training loop')
    step = 0
    sampling_list = None
    for epoch in range(p.n_epoch):
        lane_agent.training_mode()
        for inputs, target_lanes, target_h, test_image, data_list in loader.Generate(sampling_list):

            #util.visualize_points(inputs[0], target_lanes[0], target_h[0])
            #training
            print("epoch : " + str(epoch))
            print("step : " + str(step))
            loss_p = lane_agent.train(inputs, target_lanes, target_h, epoch, lane_agent, data_list)
            torch.cuda.synchronize()
            loss_p = loss_p.cpu().data
            
            if step%50 == 0:
                vis.line(
                    X=torch.ones((1, 1)).cpu() * int(step/50),
                    Y=torch.Tensor([loss_p]).unsqueeze(0).cpu(),
                    win=loss_window,
                    update='append')
                
            if step%100 == 0:
                lane_agent.save_model(int(step/100), loss_p)
                testing(lane_agent, test_image, step, loss_p)
            step += 1

        sampling_list = copy.deepcopy(lane_agent.get_data_list())
        lane_agent.sample_reset()

        #evaluation
        if epoch%1 == 0:
            print("evaluation")
            lane_agent.evaluate_mode()
            th_list = [0.9]
            index = [3]
            lane_agent.save_model(int(step/100), loss_p)

            for idx in index:
                print("generate result")
                test.evaluation(loader, lane_agent, index = idx, name="test_result_"+str(epoch)+"_"+str(idx)+".json")
                name = "epoch_idx_"+str(epoch) + str(idx) + str(step/100)
                os.system("sh /home/kym/research/autonomous_car_vision/lane_detection/code/ITS/CuLane/evaluation_code/SCNN_Pytorch/utils/lane_evaluation/CULane/Run.sh " + name)

        if int(step)>700000:
            break


def testing(lane_agent, test_image, step, loss):
    lane_agent.evaluate_mode()

    _, _, ti = test.test(lane_agent, np.array([test_image]))

    cv2.imwrite('test_result/result_'+str(step)+'_'+str(loss)+'.png', ti[0])

    lane_agent.training_mode()

    
if __name__ == '__main__':
    Training()

