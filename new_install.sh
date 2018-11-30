#!/bin/bash

get_aravis(){	
	wget http://ftp.acc.umu.se/pub/GNOME/sources/aravis/0.5/aravis-0.5.7.tar.xz;
	tar -xf aravis-0.5.7.tar.xz
	cd aravis-0.5.7
	./configure
	make
	sudo make install
	cd ..
	rm aravis-0.5.7.tar.xz
}
get_opencv(){ 	
	echo $PWD
	mkdir OpenCV
	cd OpenCV
	#git clone https://github.com/opencv/opencv.git
	wget https://github.com/opencv/opencv/archive/3.4.1.tar.gz
	tar -xzvf 3.4.1.tar.gz
	cd opencv-3.4.1
	mkdir release
	cd release
	cd ..
	cd ..
	cd ..
	move_files 
	cd OpenCV/opencv-3.4.1/release
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_ARAVIS=ON -D CMAKE_INSTALL_PREFIX=/usr/local ..
	make
	sudo make install
	cd ..
	cd ..
	cd ..
}
get_cuda(){		
	wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
	sudo apt purge nvidia*
	sudo sh ./cuda_8.0.61_375.26_linux-run --kernel-source-path=/usr/src/linux-source-4.4.0
}

move_files(){
	readarray -t a < ./config.txt

	for each in "${a[@]}"
	do
		cp "replace$each" "OpenCV/opencv-3.4.1$each"
		#echo "opencv-3.4.1$each"	
	done
}
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y
sudo apt-get instal build-essential -y
apt-get install linux-source -y
apt-get source linux-image-$(uname -r) -y
apt-get install linux-headers-$(uname -r) -y

sudo apt-get install autogen autoconf libtool git -y	
sudo apt-get install intltool gcc make cmake pkg-config -y
sudo apt-get install libxml2-dev libglib2.0-dev libgtk2.0-dev -y

get_aravis
get_cuda
get_opencv
