package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

/**
 * Created by guusd on 7/20/2018.
 * for logging in the ftc to a .csv file
 * modified to working and own purposes by ALowden 2019-06-20
 * ------Example------
 * private dataLogger dataLog = new Logger("Motors4Enc",true);
 * dataLog.StartLogging();
 * dataLog.Log(new String[] {"time","Target","motor1Cnt","motor2Cnt","motor3Cnt","motor4Cnt"});
 * dataLog.StopLogging();
 */
public class DataLogger {
    private String fnameHeader = "Datafile";
    private FileWriter filewriter;
    private File file;
    private String lineEnd = "\n";
    private String seperator = ",";
    private boolean bufferToMemory = false;
    private StringBuilder sb;
    FileOutputStream fOUT = null;

    DataLogger(){
       sb = new StringBuilder();
    }
    DataLogger(String fname, boolean toMemory){
       fnameHeader = fname;
       bufferToMemory = toMemory;
        sb = new StringBuilder();
    }
    public void StartLogging(){
       try {
           //String FileName = fnameHeader + Calendar.getInstance().getTime().toString() + ".csv";
           String FileName = fnameHeader + ".csv";
           //file = new File(getPublicAlbumStorageDir(fnameHeader),FileName );
           file = new File(Environment.getExternalStorageDirectory(),FileName);
           filewriter = new FileWriter(file);
       }
        catch (IOException e){
           e.printStackTrace();
       }
    }

    /**sets the filename header */
    public void setFileName(String _teamname){
        fnameHeader = _teamname;
    }
    public void setLineEnd(String _lineEnd) {
        lineEnd = _lineEnd;
    }
    public void setSeperator(String sep) {
        seperator = sep;
    }


    /**closes the filewriter given by the id, do this to make sure your data gets saved */
    public void StopLogging() {
        try {
            if (bufferToMemory){
                filewriter.write(sb.toString());
            }
            filewriter.flush();
            filewriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /** log your data to the filewriter given by the ID */
    public void Log(String message) {
        try {
            if (bufferToMemory)
            {
                sb.append(message);
                sb.append(lineEnd);
            }
            else {
                filewriter.write(message + lineEnd);
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void Log(String[] items) {
        try {
            for (String st:items) {
                sb.append(st);
                sb.append(seperator);
            }
            sb.delete(sb.length()-seperator.length(),sb.length());
            sb.append(lineEnd);
            if (!bufferToMemory){
                filewriter.write(sb.toString());
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void Log(Double[] items) {
        try {
            for (Double db:items) {
                sb.append(db);
                sb.append(seperator);
            }
            sb.delete(sb.length()-seperator.length(),sb.length());
            sb.append(lineEnd);
            if (!bufferToMemory){
                filewriter.write(sb.toString());
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }


}
