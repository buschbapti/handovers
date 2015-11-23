import numpy as np
from scipy.interpolate import griddata

def get_table(data):
    points = data['points']
    values = data['values']
    # get the maximum index for each coordinates
    max_elem = points[-1] 
    # calculate a 2D or 3D grid depending on input
    if len(points[0]) == 2:
        grid_x, grid_y = np.mgrid[1:max_elem[0]:200j, 1:max_elem[1]:200j]
        grid_tuple = (grid_x,grid_y)
        x_index = np.array(grid_x)[:,0]
        y_index = np.array(grid_y)[0,:]
        grid_indexes = [x_index, y_index]
    else:   
        grid_x, grid_y, grid_z = np.mgrid[1:max_elem[0]:200j, 1:max_elem[1]:200j, 1:max_elem[2]:200j]
        grid_tuple = (grid_x,grid_y,grid_z)
        x_index = np.array(grid_x)[:,0,0]
        y_index = np.array(grid_y)[0,:,0]
        z_index = np.array(grid_z)[0,0,:]
        grid_indexes = [x_index, y_index, z_index]
    table = {}
    table['grid'] = griddata(points, values, grid_tuple, method='linear')
    table['indexes'] = grid_indexes
    return table

def get_value(point, table):
    def find_index(grid_indexes):
        res = []
        for i in range(len(point)):
            res.append(np.searchsorted(grid_indexes[i],point[i]))
        return np.minimum(res, len(grid_indexes[0])-1)
    # first find indexes 
    index = find_index(table['indexes'])
    # return the corresponding value from the grid
    if len(point) == 2:
        return table['grid'][index[0],index[1]]
    else:
        return table['grid'][index[0],index[1],index[2]]
