et -e

echo "Updating package list..."
sudo apt update

echo "Installing required packages for building kernel modules..."
sudo apt install -y \
	build-essential \
	dkms \
	linux-headers-$(uname -r) \
	gcc \
	make \
	perl \
	git \
	bc \
	bison \
	flex \
	libelf-dev \
	libssl-dev \
	libncurses-dev \
	wget \
        curl \
	vim \
	libfdt-dev \
	fpga-manager-xlnx \
	libncurses5-dev \
	libtinfo5 \
	xterm \
	autoconf \
	libtool \
	texinfo \
	gcc-multilib
										
echo "All necessary packages have been installed."

