import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class Music {
    
    public static void main(String[] args){
        ArrayList<String> files = new ArrayList<>();

        try {
            //Read all music files that exist
            Process search = Runtime.getRuntime().exec("ls /media/lilja/stick");
            
            BufferedReader stdInput = new BufferedReader(new 
                 InputStreamReader(search.getInputStream()));

            String s = null;

            while ((s = stdInput.readLine()) != null) {
                files.add(s);
            }

            //Shuffle songs
            Collections.shuffle(files);

            //Loop through all songs
            for (int x = 0; x < files.size(); x++){
                Process play = Runtime.getRuntime().exec("mpg123 /media/lilja/stick/" + files.get(x));
             
                //Wait until music has played
                play.waitFor();   
                
                if (x == files.size()-1){
                    x = 0;
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        } 
    }
}