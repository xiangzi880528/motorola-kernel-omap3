mkdir -p bootimg/cm/lib/modules/
cp drivers/misc/modem_pm_driver/modem_pm_driver.ko bootimg/cm/lib/modules/
cp drivers/misc/netmux/netmux.ko bootimg/cm/lib/modules/
cp drivers/misc/netmux_linkdriver/usb/netmux_linkdriver.ko bootimg/cm/lib/modules/
cp drivers/misc/sec/sec.ko bootimg/cm/lib/modules/
cp drivers/misc/wl127x_test.ko bootimg/cm/lib/modules/
cp arch/arm/boot/zImage bootimg/
echo "Copy kernel modules done!"
cd bootimg/
./repack-bootimg.pl zImage cm/ \#$(cat $(pwd)/../.version)-$(stat -c %y $(pwd)/../arch/arm/boot/zImage | cut -b1-10).img
echo "Pack boot image done!"