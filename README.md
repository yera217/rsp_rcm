# RSP Final project
It implements RCM of UR5 robot based on constrained optimization approach.

### Members: Yernar Zhetpissov (JHED id: yzhetpi1)


### Running instructions:
Dependencies:
* pyOpt [http://www.pyopt.org/install.html] (if having numpy compatibility error "TypeError: only integer scalar arrays can be converted to a scalar index": 
replace lz with lz[0] 
`/usr/local/lib/python2.7/dist-packages/pyOpt/pySLSQP/pySLSQP.py`
Lines
```
374     gg = numpy.zeros([la[0]], numpy.float)
377     dg = numpy.zeros([la[0],n+1], numpy.float)
401     w = numpy.zeros([lw[0]], numpy.float)
404     jw = numpy.zeros([ljw[0]], numpy.intc)
```
)
* gfortran
`sudo apt-get install gfortran`






### 

