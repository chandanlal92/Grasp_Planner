import numpy as np
#import shapely
#from shapely.geometry import Polygon
import tensorflow as tf

def grasp_error(grasps,targets,max_angle = 30.,min_overlap=0.25):
    '''
    i0 = tf.constant(0)
    r = tf.Variable([])
    res = tf.Variable([])
    c = lambda i, r: i < 128
    time_steps = tf.shape(targets)[0]
    def b(i, r):
        #r = tf.concat([r,grasp_classification(grasps[i],targets[i])],0)
        r = tf.stack([r,grasp_classification(grasps[i],targets[i],max_angle,min_overlap)],0)
        return i+1, r
    index, results = tf.while_loop(c,b,[i0, r],shape_invariants=[i0.get_shape(), tf.TensorShape(None)])
    res.assign(results)
    #total = tf.foldl(lambda a, x: a + x, res)

    '''
    i0 = tf.constant(0)
    time_steps = tf.shape(targets)[0]
    init_state = tf.TensorArray(tf.float32,size=time_steps)

    def c(i, *args):
        return i < time_steps

    def b(i, r):
        # r = tf.concat([r,grasp_classification(grasps[i],targets[i])],0)
        r = r.write(i, grasp_classification(grasps[i], targets[i],max_angle,min_overlap))
        return i + 1, r

    index, res = tf.while_loop(c, b, [i0, init_state])
    res = res.stack()
    #result = tf.reduce_mean(tf.cast(res,tf.float32))
    #print(result)
    return res

#compute the error of the test set
def grasp_classification(grasp,target,max_angle = 30.,min_overlap = 0.25):
    #position where sin and cosin are saved in vector
    sinpos = 4
    cospos = 5
    # use arccos to get angle from sin/cosin representation and compare difference of both with limits
    def cond0(): return 0.
    def cond1(): return 1.
    cond11 = tf.cond(tf.abs(tf.atan2(grasp[sinpos],grasp[cospos]) - tf.atan2(target[sinpos],target[cospos])) < (max_angle * 2./180.)*np.pi,cond1,cond0)
    cond2 = tf.cond(tf.logical_and((cond11 > 0.),(jaccard_index(grasp,target) > min_overlap)), cond1, cond0)
    return cond2

# computes Jaccard index of two grasping rectangeles
def jaccard_index(grasp,target):
    x = 0 #position of x coordinate
    y = 1 #positinof y coordinate
    h = 2 #position of height
    w = 3 #position of width
    sinpos = 4
    cospos = 5
    #compute three corner points of the intersection rectangle
    graspangle = tf.atan2(grasp[sinpos],grasp[cospos])/2.
    #rotation matrix
    rot1 = tf.stack([[tf.cos(graspangle),-tf.sin(graspangle)], [tf.sin(graspangle),tf.cos(graspangle)]],axis=0)
    tf.reshape(rot1,[2,2])
    #rot1 = tf.placeholder(tf.float32, shape=(2, 2))
    grasprect = tf.stack([[grasp[w]/2.,grasp[h]/2.],[grasp[w]/2.,-grasp[h]/2.],[-grasp[w]/2.,-grasp[h]/2.],[-grasp[w]/2.,grasp[h]/2.]],axis=0)
    tf.reshape(grasprect, [4, 2])
    grasprect = tf.transpose(tf.matmul(rot1, tf.transpose(grasprect)))#rotate
    coor1 = tf.stack([[grasp[x],grasp[y]],[grasp[x],grasp[y]],[grasp[x],grasp[y]],[grasp[x],grasp[y]]],axis=0)
    coor1 = tf.reshape(coor1, [4, 2])
    grasprect = tf.add(grasprect,coor1) #translate

    targetangle = tf.atan2(target[sinpos],target[cospos])/2.
    rot2 = tf.stack([[tf.cos(targetangle),-tf.sin(targetangle)],[tf.sin(targetangle),tf.cos(targetangle)]],axis=0)
    tf.reshape(rot2, [2, 2])
    targetrect = tf.stack([[target[w]/2.,target[h]/2.],[target[w]/2.,-target[h]/2.],[-target[w]/2.,-target[h]/2.],[-target[w]/2.,target[h]/2.]],axis=0)
    tf.reshape(targetrect, [4, 2])
    targetrect = tf.transpose(tf.matmul(rot2, tf.transpose(targetrect)))
    coor2 = tf.stack([[target[x],target[y]],[target[x],target[y]],[target[x],target[y]],[target[x],target[y]]],axis=0)
    coor2 = tf.reshape(coor2, [4, 2])
    targetrect = tf.add(targetrect, coor2)

    x_intersection =  tf.minimum(grasprect[0][0], targetrect[0][0]) - tf.maximum(grasprect[2][0], targetrect[2][0])
    y_intersection =  tf.minimum(grasprect[0][1], targetrect[0][1]) - tf.maximum(grasprect[2][1], targetrect[2][1])
    intersect = x_intersection * y_intersection
    #print (intersect)
    overall = grasp[h] * grasp[w] + target[h] * target[w] - intersect
    is_not_intersect = tf.logical_or(x_intersection <= 0, y_intersection <= 0)
    def cond(): return 0.
    def iou(): return intersect/overall
    results = tf.cond(is_not_intersect, cond, iou)
    return results
