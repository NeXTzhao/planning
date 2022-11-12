## Interfacing with IPOPT through CMAKE 

The IPOPT library comes with a makefile that compiles a number of examples. I wanted to incorporate IPOPT and these are just my notes on the interface.

### 1) IPOPT can be downloaded with:

git clone -b stable/3.12 https://github.com/coin-or/Ipopt.git CoinIpopt

### 2) Before compiling IPOPT you need hsl. It is not open source, but a free copy can be requested from:

http://www.hsl.rl.ac.uk/ipopt/

### 3) Compile and install hsl as follows:

a) extract the copresses binaries into some directory DIR/

b) next move the files from DIR/include into usr/local/include and DIR/lib into usr/local/lib and DIR/lib/pkgconfig into usr/local/pkgconfig

c) run ldconfig to map the package names to the directory:

sudo ldconfig

### 4) Now compile and install IPOPT with the following terminal commands:

cd .../CoinIpopt

./configure

make

make install

make test

### 5) Make sure the ipopt binaries that end up in /CoinIpopt/lib/ are in /usr/local/lib and copies of related headers from /CoinIpopt/include/coin/ are copied to /usr/local/include/coin. 

