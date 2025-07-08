# Inverter Linux Kernel Module

## Overview

This kernel module provides a character device driver that maps a memory-mapped hardware peripheral (named `inverter`) into the Linux kernel address space. It supports read and write access to the mapped region using the standard character device interface and obtains device information via device tree probing.

The module is specifically written for platforms (such as Xilinx FPGAs or SoCs) that expose a memory-mapped register interface described in the device tree.

---

## Features

- Probes the device tree for hardware with the compatible string: `"xlnx,inverter_1.0"`.
- Maps the physical memory region using `ioremap()`.
- Exposes a character device (`/dev/inverter`) for userspace interaction.
- Supports 32-bit aligned reads and writes via `read()` and `write()`.
- Handles clean initialization and removal of resources.

---

## Device Tree Integration

The driver expects a device tree node with the following format:

```dts
inverter@43C00000 {
    compatible = "xlnx,inverter_1.0";
    reg = <0x43C00000 0x1000>; // Replace with the actual base address and size
};
Build and Load
Prerequisites
Linux headers installed for your running kernel.

Device tree includes a node matching the compatible string: "xlnx,inverter_1.0".

1. Build the Module
Use the following command to compile the module:

bash
Copia
Modifica
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
This assumes your module source file is named inverter.c and located in the current directory.

2. Load the Module
bash
Copia
Modifica
sudo insmod inverter.ko
To verify it loaded correctly:

bash
Copia
Modifica
dmesg | grep inverter
Expected output:

lua
Copia
Modifica
inverter: module loaded, mapped 0x1000 bytes at phys 0x43c00000, virt ffff...
3. Create the Device Node (If Needed)
If /dev/inverter is not automatically created:

bash
Copia
Modifica
sudo mknod /dev/inverter c <major_number> 0
You can find the <major_number> in dmesg output or by checking:

bash
Copia
Modifica
cat /proc/devices | grep inverter
4. Test Read/Write Access
Read a 32-bit value:

bash
Copia
Modifica
sudo dd if=/dev/inverter bs=4 count=1 skip=0
Write a 32-bit value (example: write 0xDEADBEEF):

bash
Copia
Modifica
printf '\xef\xbe\xad\xde' | sudo dd of=/dev/inverter bs=4 count=1 seek=0
Note: Only aligned 4-byte accesses are supported. Misaligned or out-of-bounds access returns -EINVAL.

Unload the Module
bash
Copia
Modifica
sudo rmmod inverter
This will clean up the memory mapping and unregister the character device.

File Descriptions
inverter.c: Main kernel module source file.

Implements device open/close (open, release), memory-mapped read/write, and device tree probing.

Uses ioremap() and copy_to_user()/copy_from_user() for safe user interaction.

Limitations
Only supports 32-bit word-aligned reads/writes.

No support for ioctl or interrupt-driven features.

Assumes the reg property in the device tree contains exactly two 32-bit values: physical base address and size.

License
This project is licensed under the GNU General Public License v2.

Author
Giuseppe Satta
