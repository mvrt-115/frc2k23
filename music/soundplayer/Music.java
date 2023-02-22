import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;

public class Music {
    
    public static void main(String[] args){
        ArrayList<String> files = new ArrayList<>();

        try {
            Process search = Runtime.getRuntime().exec("ls /media/lilja/stick");
            
            BufferedReader stdInput = new BufferedReader(new 
                 InputStreamReader(search.getInputStream()));

            String s = null;

            while ((s = stdInput.readLine()) != null) {
                files.add(s);
            }

            for (int x = 0; x < files.size(); x++){
                Process play = Runtime.getRuntime().exec("mpg123 /media/lilja/stick/" + files.get(x));
                
                //This is an excelent way to do this
                while (!processIsTerminated(play)){
                    //Wait for process to finish...
                }            
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static boolean processIsTerminated (Process process) {
        try {
            process.exitValue();
        } catch (IllegalThreadStateException itse) {
            return false;
        }
        return true;
    }
}