import java.io.File;
import java.util.Arrays;
import java.util.Collections;

public class Music {

    public static void main(String[] args){
        File folder = new File("music");
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

}
