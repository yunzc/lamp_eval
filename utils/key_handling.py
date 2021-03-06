import sys
import os
import ctypes
import numpy

keyBits = ctypes.sizeof(ctypes.c_uint64) * 8
chrBits = ctypes.sizeof(ctypes.c_ubyte) * 8
indexBits = keyBits - chrBits
chrMask = numpy.uint64(numpy.int64(~numpy.ubyte(0)) << indexBits)
indexMask = ~chrMask


def split_pg_key(pg_key):
    c_ = chr(numpy.ubyte(numpy.int64(numpy.uint64(pg_key) & chrMask) >> indexBits))
    j_ = numpy.uint64(pg_key) & indexMask
    return c_, j_
# In [79]: split_pg_key(7061644215716937728)
# Out[79]: ('b', 0)


def join_pg_key(c_, j_):
    return numpy.uint64(ord(c_) << indexBits) + numpy.uint64(j_)
# In [8]: join_pg_key('b',0)
# Out[8]: 7061644215716937728

name_to_prefix = {"husky1": 'a', "husky2": 'b', "husky3": 'c', "husky4": 'd', "spot1": 'e', "spot2": 'f', "spot3": 'g', "spot4": 'h'}
prefix_to_name = {'a': "husky1", 'b': "husky2", 'c': "husky3", 'd': "husky4", 'e': "spot1", 'f': "spot2", 'g': "spot3", 'h': "spot4"}