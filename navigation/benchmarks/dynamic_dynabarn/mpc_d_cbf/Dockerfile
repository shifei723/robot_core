FROM barn-jazzy:local
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
RUN python3 -m pip install --no-cache-dir --no-deps --target=/opt/mpc-deps casadi==3.7.2
ENV PYTHONPATH=/opt/mpc-deps
RUN python3 -c "import casadi as c; x=c.MX.sym('x'); s=c.nlpsol('smoke','ipopt',{'x':x,'f':(x-1)**2},{'ipopt.print_level':0,'print_time':False}); assert abs(float(s(x0=0)['x'])-1)<1e-6"
