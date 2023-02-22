import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

public class Music {
    
    public static void main(String[] args){
        String s = null;
        ArrayList<String> files = new ArrayList<>();

        try {
            Process p = Runtime.getRuntime().exec("ls /media/lilja/stick");
            
            BufferedReader stdInput = new BufferedReader(new 
                 InputStreamReader(p.getInputStream()));

            // read the output from the command
            while ((s = stdInput.readLine()) != null) {
                files.add(s);
            }

        } catch (IOException e) {
            e.printStackTrace();
        }

        
    }
}