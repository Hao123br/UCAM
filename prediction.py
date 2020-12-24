from statsmodels.tsa.arima_model import ARIMA                                                   
from sklearn.metrics import mean_squared_error
from pandas.plotting import autocorrelation_plot
from pykalman import KalmanFilter
import matplotlib.pyplot as plt
import pylab as pl
import numpy as np
import math
import argparse
from collections import namedtuple

seconds_to_predict = 2
order = 0,1,1
#print(order)
def arima(observations):
    model = ARIMA(observations, order=order) # ARIMA(p,d,q) --> find these values, if they fit for my use case.
    model_fit = model.fit(disp=0)
    output = model_fit.forecast(seconds_to_predict)
    # print(model_fit.summary())
    return output[0][-1]

def kalman(observations):
    masked = np.ma.append(observations, [0.] * seconds_to_predict)
    stop = len(masked)
    start =  stop - seconds_to_predict
    for i in range(start, stop):
        masked[i] = np.ma.masked
    kf = KalmanFilter(transition_matrices=np.array([[1, 1], [0, 1]]),
                        transition_covariance=0.01 * np.eye(2))
    states_pred_x = kf.em(masked).smooth(masked)[0]
    return states_pred_x[-1,0]

Position = namedtuple('Position', ['x', 'y'])

parser = argparse.ArgumentParser(description='UE position prediction script.')
parser.add_argument('-e', '--error', action='store_true',
                    help='Calculate mean squared error of algorithms. Should be used only after simulation completion')
args = parser.parse_args()

lines = open('victims_positions_log.txt','r').readlines()
num_nodes = int(lines[0])
id_offset = int(lines[1].split(',')[1])
nodes = [None] * num_nodes

for line in lines[1:]:
    time, nodeN, X, Y = line.split(',')
    nodeN = int(nodeN) - id_offset
    X = float(X)
    Y = float(Y)
    pos = Position(X,Y)
    if nodes[nodeN] is None:
        nodes[nodeN] = []
    nodes[nodeN].append(pos)

if args.error:
    mse_arima = []
    mse_kalman = []

    size = 10
    print('train initial size: {}'.format(size))
    for nodeN in range(num_nodes):
        positions = nodes[nodeN]
        Xs = [p.x for p in positions]
        train, test = Xs[:size], Xs[size+seconds_to_predict-1:]
        predictions_arima = []
        predictions_kalman = []
        
        for x in test:
            output_arima = arima(train)
            output_kalman = kalman(train)
            predictions_arima.append(output_arima)
            predictions_kalman.append(output_kalman)
            train.append(x)

        error_arima = mean_squared_error(predictions_arima, test)
        error_kalman = mean_squared_error(predictions_kalman, test)
        mse_arima.append(error_arima)
        mse_kalman.append(error_kalman)
    print('arima')
    print(mse_arima)
    print('kalman')
    print(mse_kalman)

else:
    time = int(time) + seconds_to_predict
    with open('prediction_log.txt','a') as log:
        for nodeN in range(num_nodes):
            positions = nodes[nodeN]
            Xs = [p.x for p in positions]
            Ys = [p.y for p in positions]
            prediction_x = arima(Xs)
            prediction_y = arima(Ys)
            node_id = nodeN + id_offset
            #The space at the end is important to ease spliting later
            print('{} {} {} '.format(node_id, prediction_x, prediction_y))
            log.write('{} {} {} {} \n'.format(time, node_id, prediction_x, prediction_y))

