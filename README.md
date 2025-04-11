# module-shm

MBDyn module that provides in/out data communication by shared memory.

## Getting started
### 1. Clone repository
Firstly clone MBDyn repo
```
git clone https://public.gitlab.polimi.it/DAER/mbdyn.git
cd mbdyn
git checkout develop
```
Secondly clone module repo in modules directory
```
cd ~/mbdyn/modules
git clone https://gitlab.rtc.ru/sector221/screepa/module-shm.git
```

### 1. Install MBDyn
#### Install required packages
```
sudo apt install libltdl-dev liblapack-dev libsuitesparse-dev libnetcdf-dev libnetcdf-c++4-dev
apt install autoconf automake libtool autotools-dev
```
if package `libnetcdf-c++4-dev` doesn't work -> it means that changed version number to `libnetcdf-c++5-dev` or something like that

```
sudo apt-get install mercurial \
libsuitesparse-dev libarpack2-dev libmumps-seq-dev \
libmetis-dev libnlopt-dev \
trilinos-all-dev libopenmpi-dev libptscotch-dev libsuitesparse-dev libqrupdate-dev
```
#### Configure MBDyn
bootstrap - `sh bootstrap.sh` 

default using config
```
cd mbdyn
./configure --with-static-modules \
--enable-octave \
--disable-Werror CXXFLAGS="-Ofast -Wall -march=native -mtune=native" \
CPPFLAGS="-I/usr/include/mkl -I/usr/lib/x86_64-linux-gnu/openmpi/include -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/ompi/mpi/cxx -I/usr/include/trilinos -I/usr/include/suitesparse" \
LDFLAGS="-L/usr/lib/x86_64-linux-gnu/hdf5/openmpi" \
--with-arpack --with-umfpack --with-klu --with-arpack --with-lapack --without-metis \
--with-mpi --with-trilinos --with-pardiso --with-suitesparseqr --with-qrupdate \
--enable-multithread --with-threads --with-rt
```
add `` --enable-runtime-loading --with-module=shm``
to config whith shared memory module 

For python module you may need to install some other packages (I don't remeber exactly) (☉ ‿ ⚆)

so u get something like this
```
cd mbdyn
./configure --with-static-modules \
--enable-octave \
--disable-Werror CXXFLAGS="-Ofast -Wall -march=native -mtune=native" \
CPPFLAGS="-I/usr/include/mkl -I/usr/lib/x86_64-linux-gnu/openmpi/include -I/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi/ompi/mpi/cxx -I/usr/include/trilinos -I/usr/include/suitesparse" \
LDFLAGS="-L/usr/lib/x86_64-linux-gnu/hdf5/openmpi" \
--with-arpack --with-umfpack --with-klu --with-arpack --with-lapack --without-metis \
--with-mpi --with-trilinos --with-pardiso --with-suitesparseqr --with-qrupdate \
--enable-multithread --with-threads --with-rt \
--enable-runtime-loading --with-module=shm
```

Finally, 
```
make -j20
sudo make install 
```
if you got an error when launch mbdyn file `ModuleLoad_int: unable to open module <libmodule-shm> (file not found)`. Try change `with-module=shm` to `with-module='shm template2'`


## Using
WARNING: now only supports joints to output 	༼つಠ益ಠ༽つ

I hope you already now how to use MBDyn input files ( ͡° ͜ʖ ͡°)

So to use module you need to add in input file:
#### Before drivers: `module load: module-shm` - load module to work
#### In drivers section: 
```
file: 10,       # - driver name
shared memory,  # - driver type
3,              # - num drivers
0,              # - initial time
0.01,           # - time step
"in";           # - shared memory name
```
It  works analogically as `fixed step driver` or `socket stream drive`, but with `shared memory`

#### In elements section: 
```
user defined: 1000,     # - element name
		shm_out,        # - element type
		"out",          # - shared memory name
		3,              # - output num
		   1, "rz",     # - element label, "private data name"
		   2, "rz",
		   3, "rz"; 
```
Notice, input driver and output can work separately

#### !!!WARNING!!!
You need count shm-driver as common driver in mbdyn `control data`, 
but `shm_out` is not common mbdyn `output element`, so you mustn't count it in `control data`

Obviously, input and output shared memory names must be not equal

## Syntax highlighting

To enable syntax highlighting in project use `bear make -j20` instead of 'make -j20' and reboot

Really usefull func	ଘ(੭*ˊᵕˋ)੭* ̀