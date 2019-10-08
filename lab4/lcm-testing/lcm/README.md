## LCM build/install
to install lcm, follow pdf until step 3

$ sudo apt-get install libglib2.0-dev
$ cmake .
$ make -j4
$ sudo make install 
$ sudo ldconfig -v

## LCM multiple hosts
get address to multicast group of the network adapter in use

$ netstat -g
$ export LCM_DEFAULT_URL=udpm://<adress>:7667?ttl=1





