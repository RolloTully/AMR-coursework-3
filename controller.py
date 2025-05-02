# Implement a controller
import numpy as np
class Queue():
    '''Implements a queue cos im not sure if were allowed to use the class'''
    def __init__(self,length):
        self.length = length#the length of the queue
        self.memory = []

    def push(self, var):
        #Adds element to the queue
        #removes the oldest element if the queue length exceeds the desired length
        self.memory.append(var)
        if len(self.memory) > self.length:
            del self.memory[0]

    def __call__(self):
        #returns the memory
        return np.array(self.memory)

def update_gains(control_gains,ISE_delta, ITAE_delta):
    #Update gains
    P_adjustment = np.array([-0.05 if adj>0 else 0.05 for adj in ISE_delta ])
    D_adjustment = np.array([-0.05 if adj>0 else 0.05 for adj in ITAE_delta ])
    control_gains[0,:] *= P_adjustment+1
    control_gains[1,:] *= D_adjustment+1
    print("###################  Updated Gains ###################")
    print(control_gains)
    return control_gains

def ISE(error_array, time_array):
    '''Integral Square Error'''
    #https://www.scientificbulletin.upb.ro/rev_docs_arhiva/full0e5_577239.pdf
    return np.sum(np.square(error_array)*np.array(time_array)[:, np.newaxis],axis=0)

def ITAE(error_array, time_array):
    '''Integral Time Averaged Error'''
    #https://www.scientificbulletin.upb.ro/rev_docs_arhiva/full0e5_577239.pdf
    return np.sum(np.abs(error_array)*np.array(time_array)[:,np.newaxis],axis=0)

def DCM(matrix, yaw):
    '''Computed the directional cosine matrix to convert from the global to local corrdinate frame'''
    '''the transformation matrix has been extended to pass through th euler angles so that all errors can be handled simulatiously'''
    transform_matrix = np.array([
        [ np.cos(yaw),  np.sin(yaw), 0, 0],
        [-np.sin(yaw),  np.cos(yaw), 0, 0],
        [           0,            0, 1, 0],
        [           0,            0, 0, 1]
    ])#is this pythonic, hell no, does it make it easy to read, SOOO much
    return np.matmul(transform_matrix,matrix)
error_queue = Queue(12)
time_queue = Queue(12)
ISTE_error_queue = Queue(12)
error_queue.memory = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]#fills the error queue so that a derivative can be computed later
control_gains = np.array([[0.2,0.2,0.2,0.2],
                          [0.1,0.1,0.1,0.1]]) #Arbiraty starting PD controller gains
review_interval = 12
error_history = []
time_history = []
old_ISE_loss = np.array([1,1,1,1])
old_ITAE_loss = np.array([1,1,1,1])
def controller(state, target_pos, dt):
    global error_history, time_history, old_ISE_loss, old_ITAE_loss, control_gains
    # state format: [position_x (m), position_y (m), position_z (m), roll (radians), pitch (radians), yaw (radians)]
    # target_pos format: (x (m), y (m), z (m), yaw (radians))
    # dt: time step (s)
    # return velocity command format: (velocity_x_setpoint (m/s), velocity_y_setpoint (m/s), velocity_z_setpoint (m/s), yaw_rate_setpoint (radians/s))
    state = np.array(state)
    state = state[[True,True,True,False,False,True]].T#Turns the state vector in to a column vector for ease of handeling, we dont care about roll or pitch so we filter them out
    target = np.array(target_pos).T #repeats the previous step but for the target position, roll and pitch data are added for completions sake
    error = [0,0,0,0]
    error[:3] = target[:3]-state[:3]
    error[3] = min([target[3]-state[3],2*np.pi - target[3]-state[3]], key = abs) #we have to compute angular error very differently, this makes sure the drone turns throught the smallest angle
    print(target)
    print(state)
    print(error)
    error = target-state#the position error is comuted
    error_queue.push(error.T)#the error is pushed to the error queue
    error_history.append(error)
    time_history.append(dt)
    if len(error_history)>review_interval:
        #Calculates losses and updates control gains
        ISE_loss = ISE(error_history, time_history)
        ITAE_loss = ITAE(error_history, time_history)
        control_gains = update_gains(control_gains, ISE_loss-old_ISE_loss,ITAE_loss-old_ITAE_loss )
        error_history, time_history = [], []
        old_ISE_loss = ISE_loss
        old_ITAE_loss = ITAE_loss
    control_var = np.sum(control_gains*np.array([error_queue.memory[1],np.diff(error_queue(),axis=0)[0]*dt]),axis=0)# the PD global contral variable is computed
    '''Local Frame'''
    local_commanded_move = DCM(control_var.T,state[3])#the global control variable is moved in to the local frame
    return tuple(local_commanded_move.T)#seperates out the command variables we want to use, given that roll and pitch are controlled automatically they are ignored
