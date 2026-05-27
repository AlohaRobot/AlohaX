# AlohaX Dual Arm Serial Port ttyACM Fix Method

Normally, ttyACM0 and ttyACM1 represent AlohaX robot's left and right arm Waveshare control board ports respectively. However, in cases of USB port plugging/unplugging or interference, the port numbers of the two Waveshare control boards may change, causing software to not find left or right arm. Using the following method can fix left and right arm port names.

## One-step Check Both Control Board Info (Direct Copy)

```bash
for port in /dev/ttyACM*; do
  echo "=== Device: $port ==="
  udevadm info -n $port -a | grep -E 'KERNELS|idVendor|idProduct|serial'
done
```

## Directly Generate Fixed Alias (Copy and Paste)

1. Create rules file
```bash
sudo nano /etc/udev/rules.d/99-robot-arms.rules
```
or
```bash
sudo nano /etc/udev/rules.d/99-so101.rules
```

2. Paste content below
Replace your serial number with the example string

```
## SO101 Robot Arm - Left ARM (serial: ***) → Fixed port /dev/so101_L
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="fill_ttyACM0_serial_here", MODE="0666", SYMLINK+="so101_L"

## SO101 Robot Arm - Right ARM (serial: ***) → Fixed port /dev/so101_R
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="fill_ttyACM1_serial_here", MODE="0666", SYMLINK+="so101_R"
```

3. Apply rules

Run
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. Verify success

Run
```bash
ls -l /dev/so101_L /dev/so101_R
```

Arrow pointing indicates success:
```
lrwxrwxrwx 1 root root 10 May 9 10:00 /dev/so101_L -> ttyACM0
lrwxrwxrwx 1 root root 10 May 9 10:00 /dev/so101_R -> ttyACM1
```
