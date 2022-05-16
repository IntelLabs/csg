import numpy as np
cimport numpy as np

cdef extern from "math.h":
    cpdef float sqrt(float x)

def pyfunc12(np.ndarray[np.float32_t, ndim=3] dx, 
           np.ndarray[np.float32_t, ndim=3] dy, 
           np.ndarray[np.float32_t, ndim=1] boundary_x, 
           np.ndarray[np.float32_t, ndim=1] boundary_y, 
           np.int_t height, 
           np.int_t width, 
           np.int_t numChannels, 
           np.int_t k): 


    cdef np.ndarray[np.float32_t, ndim=2] r = np.zeros((height, width), dtype=np.float32)
    cdef np.ndarray[np.int32_t, ndim=3] alfa= np.zeros((height, width, 2), dtype=np.int32)
    
    cdef int sizeX = width//k
    cdef int sizeY = height//k 
    cdef int px = 3*9 
    cdef int p = px
    cdef int stringSize = sizeX * p
    cdef int _i, _j, _ii,_jj, _c, _ch, maxi, _kk
    cdef float x, y, tx, ty, magnitude, mmax, dotProd
    cdef float a_x, b_x

    #memory view 
    cdef float[:,:, :] dx_view = dx
    cdef float[:,:,:] dy_view = dy
    cdef float[:,:] r_view = r
    cdef int[:,:,:] alfa_view = alfa
    
    for _j in range(1, height-1): 
        for _i in range(1, width-1): 
            _c = 0 
            x = dx_view[_j, _i, _c]
            y = dy_view[_j, _i, _c]
            r_view[_j, _i] =  sqrt(x*x + y*y)

            for _ch in range(1, numChannels): 
                tx = dx_view[_j, _i, _ch]
                ty = dy_view[_j, _i, _ch]
                magnitude = sqrt(tx*tx + ty*ty)
                if (magnitude> r_view[_j, _i]): 
                    r_view[_j, _i] = magnitude
                    _c = _ch
                    x  = tx
                    y = ty
                
            mmax = boundary_x[0]*x + boundary_y[0]*y 
            maxi = 0 
            for _kk in range(9): #NUM_SECTOR
                dotProd = boundary_x[_kk]*x + boundary_y[_kk]*y
                if(dotProd > mmax):
                    mmax = dotProd
                    maxi = _kk
                elif (-dotProd>mmax): 
                    mmax = -dotProd
                    maxi = _kk+9 
            alfa_view[_j, _i, 0]  = maxi % 9 
            alfa_view[_j, _i, 1]  = maxi
    
    cdef int [:] nearest = np.ones((k), dtype = np.int32)
    cdef float [:,:] w = np.zeros((k,2), dtype = np.float32)
    for _i in range(k//2): 
        nearest[_i] = -1
    for _i in range(k//2, k): 
        nearest[_i] = 1
    for _j in range(k//2): 
        b_x = k//2 + _j + 0.5 
        a_x = k//2 - _j - 0.5
        w[_j, 0] = 1.0/a_x * ((a_x * b_x) / ( a_x + b_x))
        w[_j, 1] = 1.0/b_x * ((a_x * b_x) / ( a_x + b_x))
        w[k-_j-1,0] = w[_j,0]
        w[k-_j-1,1] = w[_j,1]
    
    mapp = np.zeros((sizeX*sizeY*p), np.float32)
    cdef float [:] mapp_view = mapp
    for _i in range(sizeY): 
        for _j in range(sizeX): 
            for _ii in range(k): 
                for _jj in range(k): 
                    if ((_i * k + _ii > 0) and
                        (_i * k + _ii < height - 1) and 
                        (_j * k + _jj > 0) and 
                        (_j * k + _jj < width  - 1)): 
                        
                        mapp_view[_i*stringSize + _j*p + alfa[k*_i+_ii,_j*k+_jj,0]] +=  r[k*_i+_ii,_j*k+_jj] * w[_ii,0] * w[_jj,0]
                        mapp_view[_i*stringSize + _j*p + alfa[k*_i+_ii,_j*k+_jj,1] + 9] +=  r[k*_i+_ii,_j*k+_jj] * w[_ii,0] * w[_jj,0]
                        if((_i + nearest[_ii] >= 0) and (_i + nearest[_ii] <= sizeY - 1)):
                            mapp[(_i+nearest[_ii])*stringSize + _j*p + alfa[k*_i+_ii,_j*k+_jj,0]] += r[k*_i+_ii,_j*k+_jj] * w[_ii,1] * w[_jj,0]
                            mapp[(_i+nearest[_ii])*stringSize + _j*p + alfa[k*_i+_ii,_j*k+_jj,1] + 9] += r[k*_i+_ii,_j*k+_jj] * w[_ii,1] * w[_jj,0]
                        if((_j + nearest[_jj] >= 0) and (_j + nearest[_jj] <= sizeX - 1)):
                            mapp_view[_i*stringSize + (_j+nearest[_jj])*p + alfa[k*_i+_ii,_j*k+_jj,0]] += r[k*_i+_ii,_j*k+_jj] * w[_ii,0] * w[_jj,1]
                            mapp_view[_i*stringSize + (_j+nearest[_jj])*p + alfa[k*_i+_ii,_j*k+_jj,1] + 9] += r[k*_i+_ii,_j*k+_jj] * w[_ii,0] * w[_jj,1]
                        if((_i + nearest[_ii] >= 0) and (_i + nearest[_ii] <= sizeY - 1) and (_j + nearest[_jj] >= 0) and (_j + nearest[_jj] <= sizeX - 1)):
                            mapp_view[(_i+nearest[_ii])*stringSize + (_j+nearest[_jj])*p + alfa[k*_i+_ii,_j*k+_jj,0]] += r[k*_i+_ii,_j*k+_jj] * w[_ii,1] * w[_jj,1]
                            mapp_view[(_i+nearest[_ii])*stringSize + (_j+nearest[_jj])*p + alfa[k*_i+_ii,_j*k+_jj,1] + 9] += r[k*_i+_ii,_j*k+_jj] * w[_ii,1] * w[_jj,1]

    return mapp#, r,alfa

