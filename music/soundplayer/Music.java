import java.io.File;
import java.util.Scanner;

import java.util.Arrays;
import java.util.Collections;

import org.usb4java.Context;
import org.usb4java.Device;
import org.usb4java.DeviceDescriptor;
import org.usb4java.HotplugCallback;
import org.usb4java.HotplugCallbackHandle;
import org.usb4java.LibUsb;
import org.usb4java.LibUsbException;

public class Music {

    public static void main(String[] args){
        File folder = new File("\\dev\\sda");

        System.out.println(folder.getAbsolutePath());

        //Load from USB
        try {
            Scanner reader = new Scanner(folder);
        
        } catch (Exception e){
            System.out.println("Oops! Could not load from USB :(");
            System.exit(1);
        }

        File[] listOfFiles = folder.listFiles();
        SoundPlayer player = new SoundPlayer();

        Collections.shuffle(Arrays.asList(listOfFiles));

        for (int i = 0; i < listOfFiles.length; i++) {
            
            if (listOfFiles[i].isFile()) {
                System.out.println("File " + listOfFiles[i].getName());
                if(listOfFiles[i].getName().contains(".wav")){
                    player.setSong(listOfFiles[i].getName());
                    player.play();
			
		    System.out.println(player.clip.isActive());
                    while (player.clip.isActive()){
                        //wait here - this is indeed a very very good way to do it
                        //i definately should not be using conditions
                    }
                }
            }
        }
    }

    public int processEvent(Context context, Device device, int event,
        Object userData) {
        DeviceDescriptor descriptor = new DeviceDescriptor();
        int result = LibUsb.getDeviceDescriptor(device, descriptor);
        if (result != LibUsb.SUCCESS) {
            throw new LibUsbException("Unable to read device descriptor",
                result);
        }
        System.out.format("%s: %04x:%04x%n",
            event == LibUsb.HOTPLUG_EVENT_DEVICE_ARRIVED ? "Connected" :
                "Disconnected",
            descriptor.idVendor(), descriptor.idProduct());
        return 0;
    }
}