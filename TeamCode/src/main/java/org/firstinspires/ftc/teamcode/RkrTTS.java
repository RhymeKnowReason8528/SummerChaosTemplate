package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.speech.tts.TextToSpeech;
import android.util.Log;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Created by trucc on 3/1/2017.
 */

public class RkrTTS {

    private static TextToSpeech tts;

    private static boolean isInitialized = false;

    private static ArrayList<String> preInitSpeechStrings = new ArrayList<>();

    private class TTSListener implements TextToSpeech.OnInitListener {

        @Override
        public void onInit(int initStatus) {
            if(initStatus == TextToSpeech.SUCCESS) {
                Log.d("RKR", "TTS Init complete");
                tts.setLanguage(Locale.US);
                isInitialized = true;
                for (String string : preInitSpeechStrings) {
                    speakWords(string);
                }
            } else {
                Log.d("RKR", "TTS Init FAILED!");
            }
        }
    }

    public void init(Context context) {
        tts = new TextToSpeech(context, new TTSListener());
    }

    public void speakWords(String speech) {
        if(isInitialized == true) {
            tts.speak(speech, TextToSpeech.QUEUE_ADD, null);
        } else {
            preInitSpeechStrings.add(speech);
        }
    }
}
