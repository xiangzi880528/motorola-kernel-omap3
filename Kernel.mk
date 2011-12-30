mkdir -p bootimg/21/lib/modules/
cp drivers/misc/modem_pm_driver/modem_pm_driver.ko bootimg/21/lib/modules/
cp drivers/misc/netmux/netmux.ko bootimg/21/lib/modules/
cp drivers/misc/netmux_linkdriver/usb/netmux_linkdriver.ko bootimg/21/lib/modules/
cp drivers/misc/sec/sec.ko bootimg/21/lib/modules/
cp drivers/misc/wl127x_test.ko bootimg/21/lib/modules/
cp arch/arm/boot/zImage bootimg/
echo "Copy kernel modules done!"
cd bootimg/
./repack-bootimg.pl zImage 21/ \#$(cat $(pwd)/../.version)-$(stat -c %y $(pwd)/../arch/arm/boot/zImage | cut -b1-10).img
echo "Pack boot image done!"