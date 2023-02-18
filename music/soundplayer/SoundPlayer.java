import java.io.File;
import java.io.IOException;
import javax.sound.sampled.Clip;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.sound.sampled.LineUnavailableException;

public class SoundPlayer{
	//Global Variables
	public Clip clip;
	public AudioInputStream audioStream;
	
	//Sets up player
	public SoundPlayer() {
		
	}

	public void setSong(String songName){
		try {
			File file = new File("music/" + songName);
			audioStream = AudioSystem.getAudioInputStream(file);
			clip = AudioSystem.getClip();
			clip.open(audioStream);
		}
		
		catch (UnsupportedAudioFileException | IOException | LineUnavailableException e) {
			e.printStackTrace();
		}	
	}

	//Plays music
	public void play() {
		 clip.start();
	}
	
	//Pauses music 
	public void pause() {
		 clip.stop();
	}
	
	//Restarts music
	public void reset() {
		 clip.setMicrosecondPosition(0);
	}
}