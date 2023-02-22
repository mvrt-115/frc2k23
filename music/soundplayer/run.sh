#!/bin/sh

javac -d build -cp "./jars/*" HotPlug.java SoundPlayer.java
java -cp ./build:./jars/commons-lang3-3.8.1.jar:./jars/libusb4java-1.3.0-linux-x86-64.jar:./jars/usb4java-1.3.0.jar HotPlug